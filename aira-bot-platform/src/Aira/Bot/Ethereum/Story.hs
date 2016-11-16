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
import Control.Monad.IO.Class (MonadIO(..))
import Data.Text.Lazy.Builder (toLazyText)
import Data.Text.Lazy.Builder.RealFloat
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
import Aira.Registrar

selectToken :: MonadIO m => StoryT m Address
selectToken = do
    tokenName <- select "Please select token:" [["Aira ether funds"], ["other"]]
    case tokenName :: Text of
        "Aira ether funds" -> do
            res <- liftIO $ runWeb3 (resolve "AiraEtherFunds.contract")
            case res of Right a -> return a
                        Left e -> error (show e)
        _ -> question "Custom token address:"


transfer :: Story
transfer = withAddress noRegStory $ \address c -> do
    token <- selectToken
    AccountAddress destination <- question "Recipient username:"
    amount <- question "Amount of `ether` you want to send:"
    res <- liftIO $ runWeb3 $
        withFee address operationalFee amount $
            transferFrom token address destination amount
    return $ toMessage $ case res of
        Left (UserFail e) -> pack e
        Right tx -> "Success transaction " <> etherscan_tx tx
        Left _   -> "Internal error occured!"

send :: Story
send = withAddress noRegStory $ \address c -> do
    destination <- question "Recipient Ethereum address:"
    amount <- question "Amount of `ether` you want to send:"
    res <- liftIO $ runWeb3 $
        withFee address operationalFee amount $
            sendFrom address destination amount
    return $ toMessage $ case res of
        Left (UserFail e) -> pack e
        Right tx -> "Success transaction " <> etherscan_tx tx
        Left _   -> "Internal error occured!"

balance :: Story
balance = withAddress noRegStory $ \address c -> do
    token <- selectToken
    let userBalance = (,) <$> token `getBalance` address
                          <*> token `balanceOf` address
    Right (x, y) <- liftIO $ runWeb3 userBalance
    return (toMessage $ "Balance: " <> floatToText x <> " approved / "
                                    <> floatToText y <> " on contract")

secure :: Story
secure _ = return . toMessage $ T.unlines
-- TODO: Fill text description
    [ "@AiraSecureBot" ]
