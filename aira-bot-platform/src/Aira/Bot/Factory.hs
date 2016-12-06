{-# LANGUAGE FlexibleContexts #-}
-- |
-- Module      :  Aira.Bot.Factory
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- Aira Ethereum bot stories.
--
module Aira.Bot.Factory (
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

import Aira.Contract.Factory
import Aira.Bot.Common
import Aira.Account

create :: AccountedStory

create (Account{accountAddress = Nothing}) = return $
    toMessage ("Your account should be verified!" :: Text)

create (Account{accountAddress = Just address}) = do
    target <- select "What do you want to create?"
            [ ["ERC20 token"]
            , ["ERC20 token with emission"]
            , ["Ether vault contract"] ]

    name   <- question "Token name (e.g. Ethereum):"
    symbol <- question "Token symbol (e.g. ETH):"

    res <- case target :: Text of
        "ERC20 token"      -> do
            decimal <- question "Count of numbers after point (for integral set 0):"
            total <- question "Amount of tokens on your balance after creation:"
            liftIO $ runWeb3 $
                createToken "BuilderToken.contract" address name symbol decimal total

        "ERC20 token with emission" -> do
            decimal <- question "Count of numbers after point (for integral set 0):"
            total <- question "Amount of tokens on your balance after creation:"
            liftIO $ runWeb3 $
                createToken "BuilderTokenEmission.contract" address name symbol decimal total

        "Ether vault contract"   ->
            liftIO $ runWeb3 $ createTokenEther address name symbol

        _ -> return $ throwError (UserFail "Unknown target! Cancelled.")

    return $ toMessage $ case res of
        Right tx -> "Success transaction " <> etherscan_tx tx
        Left e   -> pack (show e)
