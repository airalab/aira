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
  , ident
  ) where

import Control.Monad.Error.Class (throwError)
import Network.Ethereum.Web3.Address
import Network.Ethereum.Web3
import Data.Text as T

import Web.Bot.Message
import Web.Bot.User
import Web.Bot

import Aira.TextFormat
import Aira.Registrar
import Aira.Account

instance Answer Address where
    parse (MsgText addr) =
        case fromText addr of
            Left e -> throwError (T.pack e)
            Right a -> return a
    parse _ = throwError "Please send me Ethereum address."

instance Answer Ether where
    parse msg = do
        val <- parse msg
        if val > 0 then return $ realToFrac (val :: Double)
                   else throwError "Please send me positive value."

secure :: AiraStory a
secure _ = return . toMessage $ T.unlines
-- TODO: Fill text description
    [ "@AiraSecureBot" ]

about :: AiraStory a
about (u, px : _) = return $ toMessage $ T.unlines $
    [ "Hello, " <> userName u <> "!"
    , "Your account is " <> uri_address px ]

ident :: AiraStory a
ident (u, _) = return $ toMessage $ T.unlines $
    [ "Your personal identity:"
    , userIdent u ]

start :: AiraStory a
start = about
