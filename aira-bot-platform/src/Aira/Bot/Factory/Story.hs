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

factoryFee :: Double
factoryFee = 0.12

create :: Story
create = withAddress noRegStory $ \address _ -> do
    target <- select "What do you want to create?"
            [ ["Standart token"]
            , ["Token with emission"]
            , ["Token holds Ether"] ]

    name   <- question "Token name (e.g. Ethereum):"
    symbol <- question "Token symbol (e.g. ETH):"

    -- Evaluate Web3 creation function with factory fee
    let runCreate = liftIO . runWeb3 . withFee address factoryFee 0

    res <- case target :: Text of
        "Standart token"      -> do
            decimal <- question "Count of numbers after point (for integral set 0):"
            total <- question "Amount of tokens on your balance after creation:"
            runCreate $
                createToken "BuilderToken.contract" address name symbol decimal total

        "Token with emission" -> do
            decimal <- question "Count of numbers after point (for integral set 0):"
            total <- question "Amount of tokens on your balance after creation:"
            runCreate $
                createToken "BuilderTokenEmission.contract" address name symbol decimal total

        "Token holds Ether"   ->
            runCreate $ createTokenEther address name symbol

        _ -> runCreate $ throwError (UserFail "Unknown target! Cancelled.")

    return $ case res of
        Right tx -> toMessage ("Success transaction " <> etherscan_tx tx)
        Left e   -> toMessage (pack $ show e)
