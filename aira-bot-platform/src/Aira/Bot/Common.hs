{-# LANGUAGE TypeSynonymInstances #-}
{-# LANGUAGE FlexibleInstances    #-}
{-# LANGUAGE DataKinds            #-}
-- |
-- Module      :  Aira.Bot.Common
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- Aira common bot stories.
--
module Aira.Bot.Common (
    secure
  , about
  , start
  ) where

import Control.Monad.Error.Class (throwError)
import Network.Ethereum.Web3.Address
import Network.Ethereum.Web3
import Text.Read (readMaybe)
import Web.Telegram.Bot
import Data.Text as T

import Aira.TextFormat
import Aira.Registrar
import Aira.Account

instance Answer Address where
    parse msg = case text msg of
        Nothing -> throwError "Please send me Ethereum address."
        Just addr -> case fromText addr of
            Left e -> throwError (T.pack e)
            Right a -> return a

instance Answer Ether where
    parse msg = case (readMaybe . (++ " ether") . T.unpack) =<< text msg of
        Nothing -> throwError "Please send me a value in ether."
        Just x -> if x > 0 then return x
                           else throwError "Please send me a positive value."

secure :: AiraStory
secure _ = return . toMessage $ T.unlines
-- TODO: Fill text description
    [ "@AiraSecureBot" ]

about :: AiraStory
about (u, _, px : _) = return $ toMessage $ T.unlines $
    [ "Hello, " <> user_first_name u <> "!"
    , "Your account is " <> etherscan_addr px ]

start :: AiraStory
start = about
