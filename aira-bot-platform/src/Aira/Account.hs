{-# LANGUAGE DataKinds #-}
-- |
-- Module      :  Aira.Account
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  unknown
--
-- AIRA accounting system.
--
module Aira.Account (
    Account(..)
  , AiraStory
  , accounting
  , airaWeb3
  , proxy
  ) where

import qualified Aira.Contract.BuilderProxy as BuilderProxy
import qualified Aira.Contract.Proxy        as Proxy
import qualified Data.ByteString.Base16     as B16
import qualified Data.ByteArray             as BA
import qualified Data.Text                  as T
import Data.Text.Encoding (encodeUtf8)
import Crypto.Hash (hash, Digest, Keccak_256)
import Control.Concurrent.Chan
import Control.Exception (throwIO)
import Control.Monad.IO.Class
import Control.Monad (filterM)
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
proxy a tgt val tx = Proxy.request a nopay tgt (toWei val) (toBytes tx)
  where toBytes = BytesD . BA.convert . fst . B16.decode . encodeUtf8 . toData

type Account   = (User, [Proxy])
type AiraStory = Account -> StoryT Bot BotMessage

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

-- | Load proxies by user tag
loadProxies :: MonadIO m => User -> m [Proxy]
loadProxies u = do
    Right builder <- airaWeb3 $ getAddress "BuilderProxy.contract"
    Right bot     <- airaWeb3 $ getAddress "AiraEth.bot"
    proxies <- trySeq (BuilderProxy.getContractsOf builder bot <$> [0..])
    Right upx <- airaWeb3 $
        filterM (fmap (== userIdent u) . Proxy.getIdent) proxies
    return upx

-- | User account creation story
accountCreation :: User -> StoryT Bot Account
accountCreation u = do
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
            return (u, [inst])

        Left e -> liftIO (throwIO e)

-- | User accounting combinator
accounting :: AiraStory -> Story
accounting story u = do
    px <- loadProxies u
    case px of
        [] -> accountCreation u >>= story
        _  -> story (u, px)
