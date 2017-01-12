{-# LANGUAGE DataKinds #-}
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
import Crypto.Hash (hash, Digest, Keccak_256)
import Data.Text.Encoding (encodeUtf8)
import Control.Exception (throwIO)
import Control.Monad.IO.Class

import Web.Telegram.Bot.Types
import Web.Telegram.Bot

import Network.Ethereum.Web3.Encoding
import Network.Ethereum.Web3.Types
import Network.Ethereum.Web3

import Aira.TextFormat
import Aira.Registrar
import Aira.Config
import Pipes (yield)

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
proxy a b c d = do feeGuard a
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
createProxy :: User -> StoryT (Bot AiraConfig) Proxy
createProxy u = do
    yield $ toMessage $ T.unlines
        [ "Hello, " <> user_first_name u <> "!"
        , "New user initiated, please wait..." ]
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
            yield $ toMessage $ "Proxy initiated at " <> etherscan_tx tx
                             <> ", waiting for confirmation..."
            inst <- liftIO (readChan notify)
            yield $ toMessage $ "Proxy instantiated as " <> etherscan_addr inst <> "!"
            return inst

        Left e -> liftIO (throwIO e)

proxyNotifyBot :: Bot AiraConfig ()
proxyNotifyBot = do
    proxies <- liftIO newChan

    -- Proxy listen spawner
    forkBot $
        forever $
            liftIO (readChan proxies) >>= proxyListener

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

proxyListener :: Address -> Bot AiraConfig ()
proxyListener = undefined
