{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE TypeFamilies    #-}
{-# LANGUAGE DataKinds       #-}
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
  ) where

import qualified Aira.Contract.BuilderProxy as BuilderProxy
import qualified Aira.Contract.Proxy        as Proxy
import qualified Aira.Contract.Token        as ERC20
import qualified Data.ByteString.Base16     as B16
import qualified Data.ByteArray             as BA
import qualified Data.Text                  as T

import Control.Concurrent.Chan (newChan, writeChan, readChan)
import Control.Monad (filterM, when, forever)
import Data.Text.Encoding (encodeUtf8)
import Control.Exception (throwIO)
import Control.Monad.IO.Class

import Network.Ethereum.Web3.Encoding
import Network.Ethereum.Web3.Address
import Network.Ethereum.Web3.Types
import Network.Ethereum.Web3
import Data.ByteArray (Bytes)
import Database.Persist.Class

import Web.Bot.Persist
import Web.Bot.User
import Web.Bot.Log
import Web.Bot

import Aira.TextFormat
import Aira.Registrar
import Aira.Config

type Proxy = Address

toBytes :: Text -> Bytes
toBytes = BA.convert . fst . B16.decode . encodeUtf8

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
  where txData = BytesD (toBytes (toData tx))

-- | Air taxman
feeGuard :: Provider p => Address -> Web3 p TxHash
feeGuard a = do
    air <- getAddress "TokenAir.contract"
    bot <- getAddress "AiraEth.bot"
    bal <- ERC20.balanceOf air a
    alw <- ERC20.allowance air a bot
    when (bal < fee || alw < fee) $
        liftIO $ throwIO $ UserFail "Too low Air balance!"
    ERC20.transferFrom air nopay a bot fee
  where fee = 1

-- | Like @proxy'@ but take a fee and add some checks
proxy :: (Method method, Unit amount, Provider p)
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
proxy a b c d = do feeGuard a
                   -- TODO: Add checks
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
userProxies :: MonadIO m => User -> m [Proxy]
userProxies u = do
    proxies <- botProxies
    res <- airaWeb3 $
        filterM (fmap ((== userIdent u) . T.pack . show) . Proxy.getIdent) proxies
    case res of
        Left e -> liftIO (throwIO e)
        Right upx -> return upx

-- | Create first proxy for given user
createProxy :: APIToken a => User -> StoryT (Bot a) Proxy
createProxy user = do
    yield $ toMessage $ "Hello, " <> userName user <> "!"
    notify <- liftIO newChan

    res <- airaWeb3 $ do
        builder <- getAddress "BuilderProxy.contract"
        bot     <- getAddress "AiraEth.bot"
        cost    <- fromWei <$> BuilderProxy.buildingCostWei builder

        event builder $ \(BuilderProxy.Builded sender inst) ->
            if sender == bot then do
                res <- fmap (T.pack . show) <$> airaWeb3 (Proxy.getIdent inst)
                case res of
                    Left _ -> return ContinueEvent
                    Right ident -> do
                        print ident
                        if ident == userIdent user then do
                            writeChan notify inst
                            return TerminateEvent
                        else return ContinueEvent
            else return ContinueEvent

        BuilderProxy.create builder (cost :: Wei) (BytesN $ toBytes $ userIdent user) bot

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
                airtx <- ERC20.transfer air nopay inst 50
                proxy inst air nopay (ERC20.ApproveData bot 50)
                return airtx

            case res of
                Left e -> liftIO (throwIO e)
                Right tx -> yield $ toMessage $
                    "Free Air tokens credited at " <> uri_tx tx

            return inst

        Left e -> liftIO (throwIO e)

proxyNotifyBot :: (APIToken a, Persist a) => Bot a ()
proxyNotifyBot = do
    proxies <- liftIO newChan

    -- Proxy listener spawner
    forkBot $
        forever $ do
            p <- liftIO (readChan proxies)
            proxyListener p

    -- Spawn listener for builded proxies
    airaWeb3 $ do
        builder <- getAddress "BuilderProxy.contract"
        bot     <- getAddress "AiraEth.bot"
        event builder $ \(BuilderProxy.Builded sender inst) -> do
            when (sender == bot) $
                writeChan proxies inst
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
    msgQueue <- liftIO newChan

    airaWeb3 $ do
        event px $ \(Proxy.PaymentReceived sender value) -> do
            liftIO $ writeChan msgQueue $ toMessage $ T.unlines
                [ "Incoming payment:"
                , "- from    : " <> uri_address sender
                , "- value   : " <> T.pack (show (fromWei value :: Ether))
                , "- account : " <> uri_address px ]
            return ContinueEvent

        event px $ \(Proxy.CallRequest index) -> do
            res <- airaWeb3 $ do
                (tgt, val, dat, _) <- Proxy.callAt px index
                let msg = "New transaction:"
                        : txInfo px index tgt (fromWei val) dat
                ready <- Proxy.isAuthorized px index
                if not ready
                then return $ T.unlines $
                        msg ++ ["- status  : waiting for authorization"]
                else do
                    tx <- Proxy.run px nopay index
                    return $ T.unlines $
                        msg ++ [ "- status  : authorized, pending"
                               , "- hash    : " <> uri_tx tx ]
            case res of
                Left e -> print e
                Right r -> writeChan msgQueue (toMessage r)
            return ContinueEvent

        event px $ \(Proxy.CallAuthorized index authNode) -> do
            res <- airaWeb3 $ do
                let msg = "Transaction #" <> T.pack (show index)
                        <> " is authorized by " <> uri_address authNode <> "."
                (_, _, _, blk) <- Proxy.callAt px index
                ready <- (blk == 0 &&) <$> Proxy.isAuthorized px index
                if not ready
                then return $ T.unlines
                        [ msg
                        , "Transaction is not ready for run, "
                        , "may be additional authorization is needed." ]
                else do
                    tx <- Proxy.run px nopay index
                    return $ T.unlines
                        [ msg
                        , "Transaction unlocked!"
                        , "Is pending in " <> uri_tx tx ]
            case res of
                Left e -> print e
                Right r -> writeChan msgQueue (toMessage r)
            return ContinueEvent

        event px $ \(Proxy.CallExecuted index block) -> do
            res <- airaWeb3 $ do
                (tgt, val, dat, _) <- Proxy.callAt px index
                return $ T.unlines $
                    ("Transaction executed:"
                    : txInfo px index tgt (fromWei val) dat)
                    ++ [ "- status  : executed"
                       , "- block   : " <> uri_block block ]
            case res of
                Left e -> print e
                Right r -> writeChan msgQueue (toMessage r)
            return ContinueEvent

    forever $ do
        msg <- liftIO (readChan msgQueue)
        withUser $ flip sendMessage msg
  where
    withUser f = do
        res <- fmap (T.pack . show) <$> airaWeb3 (Proxy.getIdent px)
        case res of
            Left e -> $logErrorS "Proxy" (T.pack $ show e)
            Right ident -> do
                mbUser <- runDB $ getBy (UserIdentity ident)
                case mbUser of
                    Just user -> f (entityVal user)
                    Nothing ->
                        $logErrorS "Proxy" ("Unknown user with ident: " <> ident)
