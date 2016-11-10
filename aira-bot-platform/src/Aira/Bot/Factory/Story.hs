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
module Aira.Bot.Factory.Story (
    create
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

import Aira.Bot.Factory.Contract
import Aira.Bot.Story

create :: Story
create = withUsername noName
       $ withAddress noRegStory
       $ \address _ -> do
           target <- select "What do you want to create?"
                     [ ["Standart token"]
                     , ["Token with emission"] ]
           name    <- question "Token name (e.g. Ethereum):"
           symbol  <- question "Token symbol (e.g. ETH):"
           decimal <- question "Count of numbers after point (for integral set 0):"
           total <- question "Amount of tokens on your balance after creation:"
           let contract = case target :: Text of
                    "Standart token" -> "Builder2Token.contract"
                    _                -> "Builder2TokenEmission.contract"
           res <- liftIO $ runWeb3 $
               withFee address 0 $
                   createToken contract address name symbol decimal total
           case res of
               Right tx ->
                   return $ toMessage ("Success transaction " <> etherscan_tx tx)
               Left e ->
                   return $ toMessage (pack $ show e)
