{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE MultiParamTypeClasses      #-}
{-# LANGUAGE TemplateHaskell            #-}
{-# LANGUAGE TypeFamilies               #-}
{-# LANGUAGE QuasiQuotes                #-}
{-# LANGUAGE DataKinds                  #-}
{-# LANGUAGE GADTs                      #-}
-- |
-- Module      :  Aira.Bot.Proxy
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- Aira account with proxy approach.
--
module Aira.Bot.Proxy (
    Proxy
  , createProxy
  , userProxies
  , proxyNotifyBot
  , proxy'
  , proxy
  , ethBalance
  ) where

import qualified Aira.Contract.BuilderProxy as BuilderProxy
import qualified Aira.Contract.Proxy        as Proxy
import qualified Aira.Contract.Token        as ERC20
import qualified Data.ByteString.Base16     as B16
import qualified Data.ByteArray             as BA
import qualified Data.Text                  as T

import Control.Concurrent.Chan (newChan, writeChan, readChan)
import Network.Ethereum.Web3.Types (CallMode(Latest))
import Network.Ethereum.Web3.Api (eth_getBalance)
import Control.Monad (filterM, when, forever)
import Data.Text.Encoding (encodeUtf8)
import Data.Text.Read (hexadecimal)
import Control.Exception (throwIO)
import Control.Monad.IO.Class

import Network.Ethereum.Web3.Encoding
import Network.Ethereum.Web3.Address
import Network.Ethereum.Web3.Types
import Network.Ethereum.Web3
import Data.ByteArray (Bytes)

import Web.Bot.Persist
import Web.Bot.User
import Web.Bot.Log
import Web.Bot

import Database.Persist.Class
import Database.Persist.Sql
import Database.Persist.TH
import Database.Persist

import Aira.TextFormat
import Aira.Registrar
import Aira.Config

type Proxy = Address

share [mkPersist sqlSettings, mkMigrate "migrateUserProxy"] [persistLowerCase|
UserProxy
    ident Text
    proxy Text
|]

toBytes :: Text -> Bytes
toBytes = BA.convert . fst . B16.decode . encodeUtf8

ethBalance :: (Provider a, Unit u) => Address -> Web3 a u
ethBalance address = do
    res <- eth_getBalance address Latest
    case hexadecimal res of
        Right (x, _) -> return (fromWei x)
        Left e       -> liftIO $ throwIO (ParserFail e)

-- | Passthrough given method of target over proxy contract
proxy' :: (Method method, Unit amount, Provider p)
      => Proxy
      -- ^ Proxy address
      -> Address
      -- ^ Target contract address
      -> amount
      -- ^ Transaction value
      -> method
      -- ^ Target contract method
      -> Web3 p TxHash
      -- ^ Transaction hash
proxy' a tgt val tx = Proxy.request a nopay tgt (toWei val) txData
  where txData = BytesD $ toBytes $ T.drop 2 (toData tx)

-- | Air taxman
feeGuard :: Provider p => Address -> Web3 p TxHash
feeGuard a = do
    air <- getAddress "TokenAir.contract"
    bot <- getAddress "AiraEth.bot"
    bal <- ERC20.balanceOf air a
    when (bal < fee) $
        liftIO $ throwIO $ UserFail "Too low Air balance! Required at least of 1!"
    proxy' a air nopay $ ERC20.TransferData bot fee
  where fee = 1

-- | Like @proxy'@ but take a fee and add some checks
proxy :: (Method method, Ord amount, Unit amount, Provider p)
      => Proxy
      -- ^ Proxy address
      -> Address
      -- ^ Target contract address
      -> amount
      -- ^ Transaction value
      -> method
      -- ^ Target contract method
      -> Web3 p TxHash
      -- ^ Transaction hash
proxy a b c d = do proxyBal <- ethBalance a
                   when (proxyBal < c) $
                       liftIO $ throwIO $
                           UserFail $ "Too low ETH balance! Required " ++ show (convert c :: Ether)
                   feeGuard a -- TODO: Add checks
                   proxy' a b c d

trySeq :: MonadIO m => [Web3 AiraConfig b] -> m [b]
trySeq = go []
  where go acc [] = return (reverse acc)
        go acc (x : xs) = airaWeb3 x
                      >>= either (\_ -> go acc [])
                                 (\a -> go (a : acc) xs)

botProxies :: MonadIO m => m [Proxy]
botProxies = do
    res <- airaWeb3 $ do
        builder <- getAddress "BuilderProxy.contract"
        bot     <- getAddress "AiraEth.bot"
        return (BuilderProxy.getContractsOf builder bot <$> [0..])
    case res of
        Left e -> liftIO (throwIO e)
        Right list -> trySeq list

-- | Load proxies by user tag
userProxies :: Persist a => User -> Bot a [Proxy]
userProxies u = do
    list <- runDB $ selectList [UserProxyIdent ==. userIdent u] []
    let parseVal = fromText . userProxyProxy . entityVal
    case mapM parseVal list of
        Left e -> do $logErrorS "Proxy" (T.pack $ show e)
                     return []
        Right r -> return r

-- | Create first proxy for given user
createProxy :: APIToken a => User -> StoryT (Bot a) Proxy
createProxy user = do
    yield $ toMessage $ T.unlines
      [ "Hello, " <> userName user <> "!"
      , "Your identity: " <> userIdent user ]

    notify <- liftIO newChan

    res <- airaWeb3 $ do
        builder <- getAddress "BuilderProxy.contract"
        bot     <- getAddress "AiraEth.bot"
        safe    <- getAddress "AiraSafe.bot"
        cost    <- fromWei <$> BuilderProxy.buildingCostWei builder

        event builder $ \(BuilderProxy.Builded sender inst) -> do
            ident <- Proxy.getIdent inst
            if (sender /= bot) || (toData ident /= userIdent user)
            then return ContinueEvent
            else do
                liftIO $ writeChan notify inst
                return TerminateEvent

        BuilderProxy.create builder (cost :: Wei)
                                    (BytesN $ toBytes $ userIdent user) safe bot

    case res of
        Right tx -> do
            yield $ toMessage $ "Account initiated at " <> uri_tx tx
                             <> ", waiting for confirmation..."
            inst <- liftIO (readChan notify)
            yield $ toMessage $ "Account instantiated as " <> uri_address inst <> "!"

            -- Greeting proxy balance
            res <- airaWeb3 $ do
                bot <- getAddress "AiraEth.bot"
                air <- getAddress "TokenAir.contract"
                ERC20.transfer air nopay inst 1 

            case res of
                Left e -> do lift $ $logErrorS "Proxy" (T.pack $ show e)
                             liftIO (throwIO e)
                Right tx -> yield $ toMessage $
                    "Bounty Air tokens credited at " <> uri_tx tx

            return inst

        Left e -> do lift $ $logErrorS "Proxy" (T.pack $ show e)
                     liftIO (throwIO e)

proxyNotifyBot :: (APIToken a, Persist a) => Bot a ()
proxyNotifyBot = do
    -- Clear DB
    runDB $ do
        runMigration migrateUserProxy
        deleteWhere [UserProxyIdent !=. "magic"]

    -- Builded proxy event queue
    proxies <- liftIO newChan

    -- Proxy listener spawner
    forkBot $
        forever $
            liftIO (readChan proxies) >>= proxyListener

    -- Spawn listener for builded proxies
    airaWeb3 $ do
        builder <- getAddress "BuilderProxy.contract"
        bot     <- getAddress "AiraEth.bot"
        event builder $ \(BuilderProxy.Builded sender inst) -> do
            when (sender == bot) $
                liftIO $ writeChan proxies inst
            return ContinueEvent

    -- Read exist proxies
    mapM_ (liftIO . writeChan proxies) =<< botProxies

txInfo :: Proxy -> Integer -> Address -> Ether -> BytesD -> [Text]
txInfo px index tgt value dat =
  [ "- account : " <> uri_address px
  , "- index   : " <> T.pack (show index)
  , "- target  : " <> uri_address tgt
  , "- value   : " <> T.pack (show value)
  , "- size    : " <> T.pack (show $ BA.length $ unBytesD dat) <> " bytes" ]

proxyListener :: (APIToken a, Persist a) => Address -> Bot a ()
proxyListener px = do
    $logDebugS "ProxyListener" (toText px <> " starting...")

    -- Make proxy event queue
    msgQueue <- liftIO newChan

    res <- airaWeb3 $ do
        event px $ \(Proxy.PaymentReceived sender value) -> do
            liftIO $ writeChan msgQueue $ T.unlines
                [ "Incoming payment:"
                , "- from    : " <> uri_address sender
                , "- value   : " <> T.pack (show (fromWei value :: Ether))
                , "- account : " <> uri_address px ]
            return ContinueEvent

        event px $ \(Proxy.CallRequest index) -> do
            (tgt, val, dat, _) <- Proxy.callAt px index
            let msg = "New transaction:"
                    : txInfo px index tgt (fromWei val) dat
            ready <- Proxy.isAuthorized px index
            if not ready
            then liftIO $ writeChan msgQueue $ T.unlines $
                        msg ++ ["- status  : waiting for authorization"]
            else do
                tx <- Proxy.run px nopay index
                liftIO $ writeChan msgQueue $ T.unlines $
                        msg ++ [ "- status  : authorized, pending"
                               , "- hash    : " <> uri_tx tx ]
            return ContinueEvent

        event px $ \(Proxy.CallAuthorized index authNode) -> do
            let msg = "Transaction #" <> T.pack (show index)
                    <> " is authorized by " <> uri_address authNode <> "."
            (_, _, _, blk) <- Proxy.callAt px index
            ready <- (blk == 0 &&) <$> Proxy.isAuthorized px index
            if not ready
            then liftIO $ writeChan msgQueue $ T.unlines
                        [ msg
                        , "Transaction is not ready for run, "
                        , "may be additional authorization is needed." ]
            else do
                tx <- Proxy.run px nopay index
                liftIO $ writeChan msgQueue $ T.unlines
                        [ msg
                        , "Transaction unlocked!"
                        , "Is pending in " <> uri_tx tx ]
            return ContinueEvent

        event px $ \(Proxy.CallExecuted index block) -> do
            (tgt, val, dat, _) <- Proxy.callAt px index
            liftIO $ writeChan msgQueue $ T.unlines $
                    ("Transaction executed:"
                    : txInfo px index tgt (fromWei val) dat)
                    ++ [ "- status  : executed"
                       , "- block   : " <> uri_block block ]
            return ContinueEvent

        Proxy.getIdent px

    case toData <$> res of
      Left e -> $logErrorS "ProxyListener" (T.pack $ show e)
      Right ident -> do
          runDB $ insert_ $ UserProxy ident (toText px)
          forkBot $
              forever $ do
                  msg <- liftIO (readChan msgQueue)
                  withUser ident $ flip sendMessage msg
          $logDebugS "ProxyListener" (toText px <> " started.")
  where
    withUser ident f = do
        mbUser <- runDB $ getBy (UserIdentity ident)
        case mbUser of
            Just user -> f (entityVal user)
            Nothing -> $logErrorS "ProxyListener"
                                  ("Unknown user with ident: " <> ident)
