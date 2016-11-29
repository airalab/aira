{-# LANGUAGE FlexibleContexts #-}
-- |
-- Module      :  Aira.Bot.Token
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- Aira Ethereum token manipulation bot stories.
--
module Aira.Bot.Token (
    transfer
  , balance
  , send
  ) where

import Control.Monad.Error.Class (throwError)
import Control.Monad.IO.Class (liftIO)
import Network.Ethereum.Web3.Address
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Web.Telegram.Bot
import Data.Text as T

import qualified Aira.Contract.AiraEtherFunds as AEF
import qualified Aira.Contract.Token as ERC20
import Aira.Bot.Common
import Aira.Registrar
import Aira.Account

transfer :: AccountedStory
transfer = selectToken transferAIRA transferERC20

balance :: AccountedStory
balance = selectToken balanceAIRA balanceERC20

selectToken :: AccountedStory
            -> AccountedStory
            -> AccountedStory
selectToken f1 f2 a = do
    tokenType <- select "Token to use:"
                        [["AIRA Ether Funds"], ["ERC20"]]
    case tokenType :: Text of
        "AIRA Ether Funds" -> f1 a
        "ERC20"            -> f2 a
        x -> return $ toMessage ("Unknown option `" <> x <> "`!")

transferERC20 :: AccountedStory
transferERC20 (Account{accountAddress = Just from}) = do
    token               <- question "Token address:"
    AccountAddress dest <- question "Recipient username:"
    amount              <- question "Value in tokens:"
    res <- liftIO $ runWeb3 $
        ERC20.transferFrom token from dest amount
    return $ toMessage $ case res of
        Right tx -> "Success " <> etherscan_tx tx
        Left e   -> pack (show e)

transferERC20 _ = return $
    toMessage ("Your account should be verified!" :: Text)

transferAIRA :: AccountedStory
transferAIRA a = do
    dest   <- question "Recipient username:"
    amount <- question "Value in `ether`:"
    res <- liftIO $ runWeb3 $
        AEF.transferFrom (accountHash a) (accountHash dest) amount
    return $ toMessage $ case res of
        Right tx -> "Success " <> etherscan_tx tx
        Left e   -> pack (show e)

balanceAIRA :: AccountedStory
balanceAIRA a = do
    let userBalance = (,) <$> AEF.getBalance (accountHash a)
                          <*> AEF.balanceOf (accountHash a)
    Right (x, y) <- liftIO (runWeb3 userBalance)
    return $ toMessage $ "Balance: " <> floatToText x <> " approved / "
                                     <> floatToText y <> " on contract"

balanceERC20 :: AccountedStory
balanceERC20 (Account{accountAddress = Just address}) = do
    token <- question "Token address:"
    Right x <- liftIO $ runWeb3 (ERC20.balanceOf token address)
    return $ toMessage $ "Balance: " <> floatToText x <> " tokens"

balanceERC20 _ = return $
    toMessage ("Your account should be verified!" :: Text)

send :: AccountedStory
send a = do
    dest   <- question "Recipient Ethereum address:"
    amount <- question "Amount of `ether` you want to send:"
    res <- liftIO $ runWeb3 $
        AEF.sendFrom (accountHash a) dest amount
    return $ toMessage $ case res of
        Right tx -> "Success " <> etherscan_tx tx
        Left e   -> pack (show e)
