{-# LANGUAGE DeriveGeneric #-}
-- |
-- Module      :  Aira.Config
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  noportable
--
-- Aira configuration.
--
module Aira.Config (
    AiraConfig
  , readConfig
  , airaWeb3
  ) where

import Web.Bot.Platform.Telegram
import Web.Bot.Platform (APIToken(..))
import Web.Bot.Persist
import Control.Monad.IO.Class
import Network.Ethereum.Web3

import Data.Aeson (FromJSON(..), ToJSON(..), Value(..))
import Data.Yaml (decodeFileEither)
import Control.Exception (throwIO)
import qualified Data.Text as T
import Data.Text (Text)
import GHC.Generics (Generic)
import Text.Read (readMaybe)

data AiraConfig = MkConfig
  { web3uri       :: String
  , database      :: Connection
  , telegramToken :: Text
  } deriving (Show, Eq, Generic)

instance FromJSON Connection where
    parseJSON (String s) =
        case readMaybe (T.unpack s) of
            Just c -> return c
            Nothing -> fail "Broken connection string!"
    parseJSON _ = fail "Connection string should be a string!"

instance ToJSON Connection where
    toJSON = toJSON . show

instance FromJSON AiraConfig
instance ToJSON AiraConfig

readConfig :: MonadIO m => m AiraConfig
readConfig = liftIO $ do
    -- Load config
    res <- decodeFileEither "aira_config.yaml"
    either throwIO return res

airaWeb3 :: MonadIO m => Web3 AiraConfig a -> m (Either Web3Error a)
{-# INLINE airaWeb3 #-}
airaWeb3 = runWeb3'

instance Provider AiraConfig where
    rpcUri = web3uri <$> readConfig

instance APIToken Telegram where
    apiToken = telegramToken <$> readConfig

instance Persist Telegram where
    persist = database <$> readConfig
