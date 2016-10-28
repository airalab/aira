{-# LANGUAGE FlexibleContexts #-}
-- |
-- Module      :  Aira.Bot.Ethereum.Story
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- Aira Ethereum bot stories.
--
module Aira.Bot.Ethereum.Story (
    transfer
  , balance
  , secure
  , send
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

transfer :: Story
transfer = withUsername noName
         $ withAddress noRegStory
         $ \address c -> do
            AccountAddress destination <- question "Recipient username:"
            amount <- question "Amount of `ether` you want to send:"
            if amount > 0 then do
                res <- liftIO $ runWeb3 (transferFrom address destination amount)
                return $ toMessage $ case res of
                    Left e   -> pack (show e)
                    Right tx -> "Transfer success with transaction " <> etherscan_tx tx
            else return $ toMessage ("I can't transfer zero tokens!" :: Text)

send :: Story
send = withUsername noName
     $ withAddress noRegStory
     $ \address c -> do
        destination <- question "Recipient Ethereum address:"
        amount <- question "Amount of `ether` you want to send:"
        if amount > 0 then do
            res <- liftIO $ runWeb3 (sendFrom address destination amount)
            return $ toMessage $ case res of
                Left e   -> pack (show e)
                Right tx -> "Send success with transaction " <> etherscan_tx tx
        else return $ toMessage ("I can't send zero tokens!" :: Text)

balance :: Story
balance = withUsername noName
        $ withAddress noRegStory
        $ \address c -> do Right amount <- liftIO $ runWeb3 (getBalance address)
                           return (toMessage $ "Avail: " <> floatToText amount)

secure :: Story
secure _ = return . toMessage $ T.unlines
-- TODO: Fill text description
    [ "@AiraSecureBot" ]
