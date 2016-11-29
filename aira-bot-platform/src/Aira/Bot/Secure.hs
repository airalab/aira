-- |
-- Module      :  Aira.Bot.Secure
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- Aira security bot stories.
--
module Aira.Bot.Secure (
    unapprove
  , unwatch
  , approve
  , watch
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
import Aira.Bot.Common
import Aira.Bot.Watch
import Aira.Contract.AiraEtherFunds
import Aira.Account
import Data.Acid

approve :: AccountedStory
approve a = do
    amount <- question "Amount of approved tokens:"
    res <- liftIO $ runWeb3 $
        secureApprove (accountHash a) amount
    return $ toMessage $ case res of
        Right tx -> "Success " <> etherscan_tx tx
        Left e -> pack (show e)

unapprove :: AccountedStory
unapprove a = do
    res <- liftIO $ runWeb3 $
        secureUnapprove (accountHash a)
    return $ toMessage $ case res of
        Right tx -> "Success " <> etherscan_tx tx
        Left e -> pack (show e)

watch :: AcidState WatchTx -> AccountedStory
watch db a = do
    res <- select "Do you want to watch incoming transactions of"
           [["self", "another"]]
    case res :: Text of
        "self" -> do
            let Just address = accountAddress a
            liftIO $ update db (WatchRecipient address (accountChat a))
            return $ toMessage ("Your address added to watch list." :: Text)
        _ -> do
            recipient <- question "Recipient address for watching:"
            liftIO $ update db (WatchRecipient recipient (accountChat a))
            return $ toMessage $ "Address " <> toText recipient
                              <> " added to watch list."

unwatch :: AcidState WatchTx -> AccountedStory
unwatch db a = do
    res <- select "Do you want to drop watcher of"
           [["self", "another"]]
    case res :: Text of
        "self" -> do
            let Just address = accountAddress a
            liftIO $ update db (UnwatchRecipient address)
            return $ toMessage ("Your address deleted from watch list." :: Text)
        _ -> do
            recipient <- question "Drop listener for address:"
            liftIO $ update db (UnwatchRecipient recipient)
            return $ toMessage $ "Address " <> toText recipient
                               <> " deleted from watch list."
