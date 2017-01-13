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
    unwatch
  , watch
  ) where

import Web.Telegram.API.Bot.Data (Chat(chat_id))
import Control.Monad.IO.Class (liftIO)
import Network.Ethereum.Web3.Address
import Aira.Bot.Common ()
import Web.Telegram.Bot
import Aira.TextFormat
import Aira.Bot.Watch
import Aira.Account
import Data.Acid

watch :: AcidState WatchTx -> AiraStory
watch db (_, c, px : _) = do
    res <- select "Do you want to watch incoming transactions of"
           [["self", "another"]]
    case res :: Text of
        "self" -> do
            liftIO $ update db $ WatchRecipient px (chat_id c)
            return $ toMessage ("Your account added to watch list." :: Text)
        _ -> do
            recipient <- question "Recipient address for watching:"
            liftIO $ update db (WatchRecipient recipient (chat_id c))
            return $ toMessage $ "Address " <> toText recipient
                              <> " added to watch list."

unwatch :: AcidState WatchTx -> AiraStory
unwatch db (_, _, px : _) = do
    res <- select "Do you want to drop watcher of"
           [["self", "another"]]
    case res :: Text of
        "self" -> do
            liftIO $ update db (UnwatchRecipient px)
            return $ toMessage ("Your account deleted from watch list." :: Text)
        _ -> do
            recipient <- question "Drop listener for address:"
            liftIO $ update db (UnwatchRecipient recipient)
            return $ toMessage $ "Address " <> toText recipient
                               <> " deleted from watch list."
