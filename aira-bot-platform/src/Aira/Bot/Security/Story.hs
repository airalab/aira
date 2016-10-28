-- |
-- Module      :  Aira.Bot.Ethereum.Story
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- Aira security bot stories.
--
module Aira.Bot.Security.Story (
    approve
  , unapprove
  ) where

import Control.Monad.Error.Class (throwError)
import Data.Text.Lazy.Builder (toLazyText)
import Data.Text.Lazy.Builder.RealFloat
import Control.Monad.IO.Class (liftIO)
import Network.Ethereum.Web3.Address
import Data.Text.Read (hexadecimal)
import Control.Applicative ((<|>))
import Data.Text.Lazy (toStrict)
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Web.Telegram.Bot
import Data.Text as T

import Aira.Bot.Activation
import Aira.Bot.Contract
import Aira.Bot.Story

approve :: Story
approve = withUsername noName
        $ withAddress noRegStory
        $ \address c -> do
            amount <- question "Amount of approved tokens:"
            res <- liftIO $ runWeb3 (secureApprove address amount)
            case res of
                Right tx -> return (toMessage $ "Success transaction " <> etherscan_tx tx)
                Left e -> return (toMessage $ pack (show e))

unapprove :: Story
unapprove = withUsername noName
          $ withAddress noRegStory
          $ \address c -> do
            res <- liftIO $ runWeb3 (secureUnapprove address)
            case res of
                Right tx -> return (toMessage $ "Success transaction " <> etherscan_tx tx)
                Left e -> return (toMessage $ pack (show e))
