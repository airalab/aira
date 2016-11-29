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

import Aira.Bot.Common (floatToText, etherscan_addr, etherscan_tx)
import Web.Telegram.Bot (forkBot, sendMessageBot, toMessage)
import Control.Concurrent (threadDelay)
import Control.Monad.IO.Class (liftIO)
import Web.Telegram.Bot.Types (Bot)
import Data.Text.Read (hexadecimal)
import Web.Telegram.API.Bot (Chat)
import Control.Monad.Reader (ask)
import qualified Data.Text as T
import qualified Data.Map as M
import Data.Maybe (fromJust)
import Control.Monad (join)
import Data.Default.Class
import Data.Monoid ((<>))
import Data.Map (Map)
import Data.SafeCopy
import Data.Acid

import Network.Ethereum.Web3.Address
import Network.Ethereum.Web3

import Lens.Family2.State
import Lens.Family2.TH
import Lens.Family2

$(deriveSafeCopy 0 'base ''Address)

data WatchTx = WatchTx { _recipient :: Map Address Chat }
  deriving Show

instance Default WatchTx where
    def = WatchTx M.empty

$(makeLenses ''WatchTx)
$(deriveSafeCopy 0 'base ''WatchTx)

watchRecipient :: Address -> Chat -> Update WatchTx ()
watchRecipient a c = recipient %= M.insert a c

unwatchRecipient :: Address -> Update WatchTx ()
unwatchRecipient a = recipient %= M.delete a

getRecipientChat :: Address -> Query WatchTx (Maybe Chat)
getRecipientChat a = recipient `views` M.lookup a <$> ask

$(makeAcidic ''WatchTx ['watchRecipient, 'getRecipientChat, 'unwatchRecipient])

eitherMaybe :: Either a b -> Maybe b
eitherMaybe (Left _) = Nothing
eitherMaybe (Right a) = Just a

handleTx :: AcidState WatchTx -> Transaction -> Bot ()
handleTx db (Transaction {txHash = hash, txFrom = from, txTo = to, txValue = value}) = do
    mbChat <- traverse (liftIO . query db . GetRecipientChat)
                       (eitherMaybe . fromText =<< to)
    case join mbChat of
        Nothing -> return ()
        Just chat -> do
            sendMessageBot chat $
                toMessage $ T.unlines [ "Incoming transaction " <> etherscan_tx hash <> ":"
                                      , "- To:    " <> etherscan_addr (fromJust to)
                                      , "- From:  " <> etherscan_addr from
                                      , "- Value: " <>
                                            case hexadecimal value of
                                                Right (v, _) ->
                                                    floatToText (fromWei v) <> " `ether`"
                                                _ -> value
                                      ]

-- | Listening new blocks and notify Tx recipents
listenBlocks :: AcidState WatchTx -> Bot ()
listenBlocks db = do
    forkBot $ do
        Right blockFilterId <- liftIO $ runWeb3 eth_newBlockFilter
        let loop = do
             res <- liftIO $ runWeb3 $ do
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
