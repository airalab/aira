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
  , UserIdent
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
import Crypto.Hash (hash, Digest, Keccak_256)
import Control.Monad.Reader.Class (ask)
import Data.Default.Class (Default(..))
import Data.Text.Encoding (encodeUtf8)
import Control.Exception (throwIO)
import Control.Monad.IO.Class

import Lens.Family2.State
import Lens.Family2.TH
import Lens.Family2

import qualified Data.Map as M
import Data.Map (Map)

import Web.Telegram.API.Bot.Data (Chat(..), ChatType(Private), User(..))
import Web.Telegram.Bot.Types
import Web.Telegram.Bot

import Network.Ethereum.Web3.Encoding
import Network.Ethereum.Web3.Types
import Network.Ethereum.Web3
import Data.ByteArray (Bytes)
import Data.SafeCopy
import Data.Acid

import Aira.TextFormat
import Aira.Registrar
import Aira.Config
import Pipes (yield)

data UserIdent = UserIdent { _identMap :: Map Text Int }
  deriving Show

instance Default UserIdent where
    def = UserIdent M.empty

$(makeLenses ''UserIdent)
$(deriveSafeCopy 0 'base ''UserIdent)

setUserChatId :: Text -> Int -> Update UserIdent ()
setUserChatId a i = identMap %= M.insert a i

getUserChatId :: Text -> Query UserIdent (Maybe Int)
getUserChatId a = identMap `views` M.lookup a <$> ask

mkChat :: Int -> Chat
mkChat cid = Chat cid Private Nothing Nothing Nothing Nothing

$(makeAcidic ''UserIdent ['setUserChatId, 'getUserChatId])

type Proxy = Address

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
proxy' a tgt val tx = Proxy.request a nopay tgt (toWei val) (toBytes tx)
  where toBytes = BytesD . BA.convert . fst . B16.decode . encodeUtf8 . toData

-- | Air taxman
feeGuard :: Provider p => Address -> Web3 p TxHash
feeGuard a = do
    air <- getAddress "AirToken.contract"
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
proxy a b c d = do --feeGuard a
                   -- TODO: Add checks
                   proxy' a b c d

trySeq :: MonadIO m => [Web3 AiraConfig b] -> m [b]
trySeq = go []
  where go acc [] = return (reverse acc)
        go acc (x : xs) = airaWeb3 x
                      >>= either (\_ -> go acc [])
                                 (\a -> go (a : acc) xs)

userIdent :: User -> BytesN 32
userIdent = BytesN . BA.convert . sha3
  where sha3 :: User -> Digest Keccak_256
        sha3 = hash . encodeUtf8 . T.pack . show

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
        filterM (fmap (== userIdent u) . Proxy.getIdent) proxies
    case res of
        Left e -> liftIO (throwIO e)
        Right upx -> return upx

-- | Create first proxy for given user
createProxy :: AcidState UserIdent -> User -> Chat -> StoryT (Bot AiraConfig) Proxy
createProxy db u c = do
    yield $ toMessage $ "Hello, " <> user_first_name u <> "!"
    notify <- liftIO newChan

    res <- airaWeb3 $ do
        builder <- getAddress "BuilderProxy.contract"
        bot     <- getAddress "AiraEth.bot"
        cost    <- fromWei <$> BuilderProxy.buildingCostWei builder

        event builder $ \(BuilderProxy.Builded sender inst) -> do
            res <- airaWeb3 $ Proxy.getIdent inst
            case res of
                Right ident -> if ident == userIdent u && sender == bot
                               then writeChan notify inst >> return TerminateEvent
                               else return ContinueEvent
                Left e -> print e >> return TerminateEvent

        BuilderProxy.create builder (cost :: Wei) (userIdent u) bot

    case res of
        Right tx -> do
            yield $ toMessage $ "Account initiated at " <> uri_tx tx
                             <> ", waiting for confirmation..."
            inst <- liftIO (readChan notify)
            yield $ toMessage $ "Account instantiated as " <> uri_address inst <> "!"

            -- Store user id
            liftIO $ update db $ SetUserChatId (T.pack $ show $ userIdent u) (chat_id c)

            -- Greeting proxy balance
            res <- airaWeb3 $ do
                air <- getAddress "AirToken.contract"
                ERC20.transfer air nopay inst 50

            case res of
                Right gtx ->
                    yield $ toMessage $ "Free Air transfered at " <> uri_tx gtx <> "."
                Left e -> liftIO (throwIO e)

            return inst

        Left e -> liftIO (throwIO e)

proxyNotifyBot :: AcidState UserIdent -> Bot AiraConfig ()
proxyNotifyBot db = do
    proxies <- liftIO newChan

    -- Proxy listener spawner
    forkBot $
        forever $ do
            p <- liftIO (readChan proxies)
            proxyListener db p

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

proxyListener :: AcidState UserIdent -> Address -> Bot AiraConfig ()
proxyListener db px = withUserChat $ \chat -> do
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
            liftIO $ writeChan msgQueue $ toMessage $
                either (T.pack . show) id res
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
            liftIO $ writeChan msgQueue $ toMessage $
                either (T.pack . show) id res
            return ContinueEvent

        event px $ \(Proxy.CallExecuted index block) -> do
            res <- airaWeb3 $ do
                (tgt, val, dat, _) <- Proxy.callAt px index
                return $ T.unlines $
                    ("Transaction executed:"
                    : txInfo px index tgt (fromWei val) dat)
                    ++ [ "- status  : executed"
                       , "- block   : " <> uri_block block ]
            liftIO $ writeChan msgQueue $ toMessage $
                either (T.pack . show) id res
            return ContinueEvent

    forkBot $
        forever $ do
            msg <- liftIO (readChan msgQueue)
            sendMessageBot chat msg
    return ()
  where
    withUserChat f = do
        res <- airaWeb3 (Proxy.getIdent px)
        case res of
            Left e -> liftIO (print e)
            Right ident -> do
                mbChatId <- liftIO $ query db (GetUserChatId (T.pack $ show ident))
                case mbChatId of
                    Just cid -> f (mkChat cid)
                    Nothing ->
                        liftIO (print $ "Unknown user with ident: " ++ show ident)
