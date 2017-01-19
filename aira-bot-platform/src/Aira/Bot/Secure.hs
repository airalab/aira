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

import Control.Monad.IO.Class (liftIO)
import Network.Ethereum.Web3.Address
import Aira.Bot.Common ()
import Aira.TextFormat
import Aira.Bot.Watch
import Aira.Account
import Web.Bot

watch :: Persist a => AiraStory a
watch (user, px : _) = do
    res <- select "Do you want to watch incoming transactions of"
           [["self", "another"]]
    case res :: Text of
        "self" -> do
            lift $ addWatch user px
            return $ toMessage ("Your account added to watch list." :: Text)
        _ -> do
            recipient <- question "Recipient address for watching:"
            lift $ addWatch user recipient
            return $ toMessage $ "Address " <> toText recipient
                              <> " added to watch list."

unwatch :: Persist a => AiraStory a
unwatch (user, px : _) = do
    res <- select "Do you want to drop watcher of"
           [["self", "another"]]
    case res :: Text of
        "self" -> do
            lift $ removeWatch user px
            return $ toMessage ("Your account deleted from watch list." :: Text)
        _ -> do
            recipient <- question "Drop listener for address:"
            lift $ removeWatch user recipient
            return $ toMessage $ "Address " <> toText recipient
                               <> " deleted from watch list."
