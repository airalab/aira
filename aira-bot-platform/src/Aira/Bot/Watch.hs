{-# LANGUAGE FlexibleInstances  #-}
{-# LANGUAGE TemplateHaskell    #-}
{-# LANGUAGE TypeFamilies       #-}
-- |
-- Module      :  Aira.Bot.Watch
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- Address watcher bot.
--
module Aira.Bot.Watch (
    UnwatchRecipient(..)
  , WatchRecipient(..)
  , WatchTx
  , listenBlocks
  ) where

import Web.Telegram.Bot (forkBot, sendMessageBot, toMessage)
import Web.Telegram.Bot.Types (Bot, BotConfig)
import Control.Concurrent (threadDelay)
import Control.Monad.IO.Class (liftIO)
import Data.Text.Read (hexadecimal)
import Web.Telegram.API.Bot (Chat(..), ChatType(Private))
import Control.Monad.Reader (ask)
import qualified Data.Text as T
import qualified Data.Map as M
import Data.Maybe (fromJust)
import Control.Monad (join)
import Data.Default.Class
import Data.Monoid ((<>))
import Aira.TextFormat
import Aira.Config
import Data.Map (Map)
import Data.SafeCopy
import Data.Acid

import Network.Ethereum.Web3.Address
import Network.Ethereum.Web3.Types
import Network.Ethereum.Web3.Api
import Network.Ethereum.Web3

import Lens.Family2.State
import Lens.Family2.TH
import Lens.Family2

$(deriveSafeCopy 0 'base ''Address)

data WatchTx = WatchTx { _recipient :: Map Address Int }
  deriving Show

instance Default WatchTx where
    def = WatchTx M.empty

$(makeLenses ''WatchTx)
$(deriveSafeCopy 0 'base ''WatchTx)

watchRecipient :: Address -> Int -> Update WatchTx ()
watchRecipient a i = recipient %= M.insert a i

unwatchRecipient :: Address -> Update WatchTx ()
unwatchRecipient a = recipient %= M.delete a

getRecipientChatId :: Address -> Query WatchTx (Maybe Int)
getRecipientChatId a = (recipient `views` M.lookup a <$> ask)

mkChat :: Int -> Chat
mkChat cid = Chat cid Private Nothing Nothing Nothing Nothing

$(makeAcidic ''WatchTx ['watchRecipient, 'getRecipientChatId, 'unwatchRecipient])

eitherMaybe :: Either a b -> Maybe b
eitherMaybe (Left _) = Nothing
eitherMaybe (Right a) = Just a

handleTx :: BotConfig a => AcidState WatchTx -> Transaction -> Bot a ()
handleTx db (Transaction {txHash = hash, txFrom = from, txTo = to, txValue = value}) = do
    mbChat <- traverse (liftIO . query db . GetRecipientChatId) to
    case join mbChat of
        Nothing -> return ()
        Just chatId -> do
            sendMessageBot (mkChat chatId) $
                toMessage $ T.unlines [ "Incoming transaction " <> uri_tx hash <> ":"
                                      , "- To:    " <> uri_address (fromJust to)
                                      , "- From:  " <> uri_address from
                                      , "- Value: " <>
                                            case hexadecimal value of
                                                Right (v, _) ->
                                                    T.pack (show (fromWei v :: Ether))
                                                _ -> value
                                      ]

-- | Listening new blocks and notify Tx recipents
listenBlocks :: BotConfig a => AcidState WatchTx -> Bot a ()
listenBlocks db = do
    forkBot $ do
        Right blockFilterId <- airaWeb3 eth_newBlockFilter
        let loop = do
             res <- airaWeb3 $ do
                hashes <- eth_getBlockFilterChanges blockFilterId
                blocks <- mapM eth_getBlockByHash hashes
                return (concatMap blockTransactions blocks)
             case res of
                Right txs -> mapM_ (handleTx db) txs
                Left e -> liftIO (print e)
             liftIO (threadDelay 10000000)
             loop
         in loop
    return ()
