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

import Network.Ethereum.Web3.Types (CallMode(Latest))
import Control.Monad.Error.Class (throwError)
import Control.Monad.IO.Class (liftIO)
import Network.Ethereum.Web3.Address
import Data.Text.Read (hexadecimal)
import Network.Ethereum.Web3.Api
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Web.Telegram.Bot
import Data.Text as T
import Control.Arrow

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
    res <- liftIO $ runWeb3 $ do
        value <- ERC20.toDecimals token amount
        ERC20.transferFrom token nopay from dest value
    return $ toMessage $ case res of
        Right tx -> "Success " <> etherscan_tx tx
        Left e   -> pack (show e)

transferERC20 _ = return $
    toMessage ("Your account should be verified!" :: Text)

transferAIRA :: AccountedStory
transferAIRA a = do
    source <- case accountAddress a of
        Nothing -> return (accountHash a)
        Just address -> do
            item <- select "Start transaction from" [["Account"], ["Linked address"]]
            case item :: Text of
                "Account" -> return (accountHash a)
                _ -> return (addressHash address)
    dest <- question "Recipient username:"
    amount <- question "Transfered value:"
    res <- liftIO $ runWeb3 $ do
        token <- getAddress "AiraEtherFunds.contract"
        AEF.transferFrom' token nopay source (accountHash dest)
                                             (toWei (amount :: Ether))
    return $ toMessage $ case res of
        Right tx -> "Success " <> etherscan_tx tx
        Left e   -> pack (show e)

ethBalance :: Unit a => Address -> Web3 a
ethBalance address = do
    res <- eth_getBalance address Latest
    case hexadecimal res of
        Right (x, _) -> return (fromWei x)
        Left e       -> throwError (ParserFail e)

balanceAIRA :: AccountedStory
balanceAIRA a = do
    res <- liftIO $ runWeb3 $ do
        botAccount <- (addressHash . Prelude.head) <$> eth_accounts
        token <- getAddress "AiraEtherFunds.contract"
        allowedBalance <- (,) <$> AEF.allowance' token (accountHash a) botAccount
                              <*> mapM (\x -> AEF.allowance' token (addressHash x) botAccount)
                                       (accountAddress a)
        totalBalance <- (,) <$> AEF.balanceOf token (accountHash a)
                            <*> mapM (AEF.balanceOf token . addressHash) (accountAddress a)
        ethers <- mapM ethBalance (accountAddress a)
        return ((fromWei *** fmap fromWei) allowedBalance,
                (fromWei *** fmap fromWei) totalBalance,
                ethers)
    return $ toMessage $ case res of
        Left e -> T.pack (show e)
        Right (allowed, total, eth) -> T.unlines $
                [ "Balances:"
                , "  Account: " <> T.pack (show (fst allowed :: Ether))
                                <> " approved / "
                                <> T.pack (show (fst total :: Ether))
                                <> " on contract" ]
                ++ case (snd allowed, snd total) of
                    (Just x, Just y) ->
                        ["  Address: " <> T.pack (show (x :: Ether))
                                       <> " approved / "
                                       <> T.pack (show (y :: Ether))
                                       <> " on contract" ]
                    _ -> []
                ++ case eth of
                    Just ethers -> ["  Ethereum: " <> T.pack (show (ethers :: Ether))]
                    _ -> []

balanceERC20 :: AccountedStory
balanceERC20 (Account{accountAddress = Just address}) = do
    token <- question "Token address:"
    Right x <- liftIO $ runWeb3 $ do
        b <- ERC20.balanceOf token address
        ERC20.fromDecimals token b
    return $ toMessage $ "Balance: " <> T.pack (show x) <> " tokens"

balanceERC20 _ = return $
    toMessage ("Your account should be verified!" :: Text)

send :: AccountedStory
send a = do
    dest   <- question "Recipient Ethereum address:"
    amount <- question "Amount of `ether` you want to send:"
    res <- liftIO $ runWeb3 $ do
        token <- getAddress "AiraEtherFunds.contract"
        AEF.sendFrom token nopay (accountHash a) dest (toWei (amount :: Ether))
    return $ toMessage $ case res of
        Right tx -> "Success " <> etherscan_tx tx
        Left e   -> pack (show e)
