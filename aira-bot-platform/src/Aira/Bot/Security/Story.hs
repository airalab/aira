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
  , watch
  , unwatch
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

import Aira.Bot.Security.Watch
import Aira.Bot.Activation
import Aira.Bot.Contract
import Aira.Bot.Story
import Data.Acid

approve :: Story
approve = withAddress noRegStory $ \address c -> do
    amount <- question "Amount of approved tokens:"
    res <- liftIO $ runWeb3 $
        withFee address operationalFee 0 $
            secureApprove address amount
    return $ toMessage $ case res of
        Left (UserFail e) -> pack e
        Right tx -> "Success transaction " <> etherscan_tx tx
        Left _   -> "Internal error occured!"

unapprove :: Story
unapprove = withAddress noRegStory $ \address c -> do
    res <- liftIO $ runWeb3 $
        withFee address operationalFee 0 $
            secureUnapprove address
    return $ toMessage $ case res of
        Left (UserFail e) -> pack e
        Right tx -> "Success transaction " <> etherscan_tx tx
        Left _   -> "Internal error occured!"

watch :: AcidState WatchTx -> Story
watch db = withAddress noRegStory $ \address c -> do
    res <- select "Do you want to watch incoming transactions of"
           [["self", "another"]]
    case res :: Text of
        "self" -> do
            liftIO $ update db (WatchRecipient address c)
            return $ toMessage ("Your address added to watch list." :: Text)
        _ -> do
            recipient <- question "Recipient address for watching:"
            liftIO $ update db (WatchRecipient recipient c)
            return $ toMessage $ "Address " <> toText recipient
                              <> " added to watch list."

unwatch :: AcidState WatchTx -> Story
unwatch db = withAddress noRegStory $ \address c -> do
    res <- select "Do you want to drop watcher of"
           [["self", "another"]]
    case res :: Text of
        "self" -> do
            liftIO $ update db (UnwatchRecipient address)
            return $ toMessage ("Your address deleted from watch list." :: Text)
        _ -> do
            recipient <- question "Drop listener for address:"
            liftIO $ update db (UnwatchRecipient recipient)
            return $ toMessage $ "Address " <> toText recipient
                               <> " deleted from watch list."
