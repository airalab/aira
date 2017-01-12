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
  , ethBalance
  ) where

import Network.Ethereum.Web3.Types (CallMode(Latest))
import Control.Monad.IO.Class (liftIO)
import Data.Text.Read (hexadecimal)
import Control.Exception (throwIO)
import Network.Ethereum.Web3.Api
import Network.Ethereum.Web3
import Web.Telegram.Bot

import qualified Aira.Contract.Token as ERC20
import qualified Data.Text           as T
import Aira.Bot.Common
import Aira.TextFormat
import Aira.Registrar
import Aira.Account

transfer :: AiraStory
transfer = selectToken transferAIRA transferERC20

balance :: AiraStory
balance = selectToken balanceAIRA balanceERC20

selectToken :: AiraStory -> AiraStory -> AiraStory
selectToken f1 f2 a = do
    tokenType <- select "Token to use:" [["Ether"], ["ERC20"]]
    case tokenType :: Text of
        "Ether" -> f1 a
        "ERC20" -> f2 a
        x -> return $ toMessage ("Unknown option `" <> x <> "`!")

transferERC20 :: AiraStory
transferERC20 (_, _, px : _) = do
    token               <- question "Token address:"
    AccountAddress dest <- question "Recipient username:"
    amount              <- question "Value in tokens:"
    res <- airaWeb3 $ do
        value <- ERC20.toDecimals token amount
        bal   <- ERC20.balanceOf token px

        if value > bal
        then liftIO $ throwIO $ UserFail $
                "Balance is too low: " ++ show bal
                                       ++ " requested: " ++ show value
        else proxy px token nopay (ERC20.TransferData dest value)

    return $ toMessage $ case res of
        Right tx -> "Success " <> etherscan_tx tx
        Left e   -> "Error " <> T.pack (show e)

transferERC20 _ = return $ toMessage $ T.unlines
    [ "Your account isn't work correctly!"
    , "Please wait on initiation step or call Airalab support." ]

transferAIRA :: AiraStory
transferAIRA (_, _, px : _) = do
    AccountAddress dest <- question "Recipient username:"
    amount              <- question "Value in ethers:"
    res <- airaWeb3 $ do
        bal <- ethBalance px

        if amount > bal
        then liftIO $ throwIO $ UserFail $
                "Balance is too low: " ++ show bal
                                       ++ " requested: " ++ show amount
        else proxy px dest (amount :: Ether) NoMethod

    return $ toMessage $ case res of
        Right tx -> "Success " <> etherscan_tx tx
        Left e   -> T.pack (show e)

transferAIRA _ = return $ toMessage $ T.unlines
    [ "Your account isn't work correctly!"
    , "Please wait on initiation step or call Airalab support." ]

ethBalance :: (Provider a, Unit u) => Address -> Web3 a u
ethBalance address = do
    res <- eth_getBalance address Latest
    case hexadecimal res of
        Right (x, _) -> return (fromWei x)
        Left e       -> liftIO $ throwIO (ParserFail e)

balanceAIRA :: AiraStory
balanceAIRA (_, _, pxs) = do
    res <- airaWeb3 $ mapM ethBalance pxs
    return $ toMessage $ case res of
        Left e -> T.pack (show e)
        Right balances -> T.unlines $
            "Account balances:" : fmap pxBalance (zip pxs balances)
  where pxBalance :: (Address, Ether) -> Text
        pxBalance (p, b) = "- " <> etherscan_addr p <> ": " <> T.pack (show b)

balanceAIRA _ = return $ toMessage $ T.unlines
    [ "Your account isn't work correctly!"
    , "Please wait on initiation step or call Airalab support." ]

balanceERC20 :: AiraStory
balanceERC20 (_, _, px : _) = do
    token <- question "Token address:"
    res <- airaWeb3 $ do
        b <- ERC20.balanceOf token px
        ERC20.fromDecimals token b
    return $ toMessage $ case res of
        Right x -> "Balance: " <> T.pack (show x) <> " tokens"
        Left e -> T.pack (show e)

balanceERC20 _ = return $ toMessage $ T.unlines
    [ "Your account isn't work correctly!"
    , "Please wait on initiation step or call Airalab support." ]

send :: AiraStory
send (_, _, px : _) = do
    dest   <- question "Recipient Ethereum address:"
    amount <- question "Amount of `ether` you want to send:"
    res <- airaWeb3 $ proxy px dest (amount :: Ether) NoMethod
    return $ toMessage $ case res of
        Right tx -> "Success " <> etherscan_tx tx
        Left e   -> T.pack (show e)

send _ = return $ toMessage $ T.unlines
    [ "Your account isn't work correctly!"
    , "Please wait on initiation step or call Airalab support." ]
