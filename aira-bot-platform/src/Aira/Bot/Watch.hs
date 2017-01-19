{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE MultiParamTypeClasses      #-}
{-# LANGUAGE TemplateHaskell            #-}
{-# LANGUAGE TypeFamilies               #-}
{-# LANGUAGE QuasiQuotes                #-}
{-# LANGUAGE GADTs                      #-}
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
    blockListenerBot
  , removeWatch
  , addWatch
  ) where

import Control.Concurrent (threadDelay)
import Control.Monad.IO.Class (liftIO)
import Control.Monad (forM_, forever)
import Data.Text.Read (hexadecimal)
import qualified Data.Text as T
import Aira.TextFormat
import Aira.Config

import Network.Ethereum.Web3.Address
import Network.Ethereum.Web3.Types
import Network.Ethereum.Web3.Api
import Network.Ethereum.Web3

import Database.Persist.Sql
import Database.Persist.TH
import Database.Persist
import Data.Text (Text)
import Web.Bot.Persist
import Web.Bot.User
import Web.Bot.Log
import Web.Bot

share [mkPersist sqlSettings, mkMigrate "migrateWatch"] [persistLowerCase|
Watch
    user    UserId
    address Text
|]

addWatch :: Persist a => User -> Address -> Bot a ()
addWatch user addr = runDB $ do
    res <- getBy $ UserIdentity (userIdent user)
    case res of
        Nothing -> $logErrorS "WatchTx" ("Unknown user: " <> userName user)
        Just u -> insert_ $ Watch (entityKey u) (toText addr)

removeWatch :: Persist a => User -> Address -> Bot a ()
removeWatch user addr = runDB $ do
    res <- getBy $ UserIdentity (userIdent user)
    case res of
        Nothing -> $logErrorS "WatchTx" ("Unknown user: " <> userName user)
        Just u ->
            mapM_ (delete . entityKey) =<<
                selectList [WatchUser ==. (entityKey u),
                            WatchAddress ==. (toText addr)] []

handleTx :: (APIToken a, Persist a) => Transaction -> Bot a ()
handleTx (Transaction {txHash = hash, txFrom = from, txTo = Just to, txValue = value}) = do
    users <- runDB $ do
        watchs <- selectList ([WatchAddress ==. toText to]
                             ||. [WatchAddress ==. toText from]) []
        mapM (getJust . watchUser) (entityVal <$> watchs)
    forM_ users $ flip sendMessage $
        T.unlines [ "Transaction " <> uri_tx hash <> ":"
                  , "- To:    " <> uri_address to
                  , "- From:  " <> uri_address from
                  , "- Value: " <>
                    case hexadecimal value of
                        Right (v, "") ->
                            T.pack (show (fromWei v :: Ether))
                        _ -> value ]
handleTx _ = return ()

-- | Listening new blocks and notify Tx recipents
blockListenerBot :: (APIToken a, Persist a) => Bot a b
blockListenerBot = do
    runDB (runMigration migrateWatch)
    Right blockFilterId <- airaWeb3 eth_newBlockFilter
    forever $ do
        res <- airaWeb3 $ do
            hashes <- eth_getBlockFilterChanges blockFilterId
            blocks <- mapM eth_getBlockByHash hashes
            return (concatMap blockTransactions blocks)
        case res of
            Right txs -> mapM_ handleTx txs
            Left e -> $logErrorS "WatchTx" (T.pack $ show e)
        liftIO (threadDelay 10000000)
