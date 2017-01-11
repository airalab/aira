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
module Aira.Config (AiraConfig, readConfig, airaWeb3) where

import Control.Monad.IO.Class
import Network.Ethereum.Web3
import Data.Aeson (FromJSON, ToJSON)
import Data.Yaml (decodeFileEither)
import Control.Exception (throwIO)
import GHC.Generics (Generic)

data AiraConfig = MkConfig
  { web3uri :: String
  } deriving (Show, Eq, Generic)

instance FromJSON AiraConfig
instance ToJSON AiraConfig

readConfig :: MonadIO m => m AiraConfig
readConfig = liftIO $ do
    -- Load config
    res <- decodeFileEither "aira_config.yaml"
    case res of
        Left e  -> throwIO e
        Right c -> return c

airaWeb3 :: MonadIO m => Web3 AiraConfig a -> m (Either Web3Error a)
{-# INLINE airaWeb3 #-}
airaWeb3 = runWeb3'

instance Provider AiraConfig where
    rpcUri = web3uri <$> readConfig
