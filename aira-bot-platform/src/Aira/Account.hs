-- |
-- Module      :  Aira.Account
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  unknown
--
-- AIRA accounting system.
--
module Aira.Account (
    AiraStory
  , Account
  , AccountAddress(..)
  , accounting
  , airaWeb3
  , proxy
  ) where

import Web.Telegram.API.Bot.Data (Chat, User, Message(text))
import Control.Monad.Error.Class (throwError)
import Control.Monad.IO.Class (liftIO)
import Network.Ethereum.Web3.Address
import qualified Data.Text as T
import Web.Telegram.Bot
import Aira.TextFormat
import Aira.Bot.Proxy
import Aira.Registrar
import Aira.Config
import Data.Acid

type Account   = (User, Chat, [Proxy])
type AiraStory = Account -> StoryT (Bot AiraConfig) BotMessage

-- | User accounting combinator
accounting :: AcidState UserIdent -> AiraStory -> Story AiraConfig
accounting db story (u, c) = do
    px <- userProxies u
    case px of
        [] -> do p <- createProxy db u c
                 story (u, c, [p])
        _  -> story (u, c, px)

newtype AccountAddress = AccountAddress Address
  deriving (Show, Eq)

instance Answer AccountAddress where
    parse msg = case text msg of
        Nothing -> throwError "Please send me text username."
        Just name -> do
            res <- liftIO $ airaWeb3 $ getAddress $ name <> ".account"
            case res of
                Left e -> throwError (T.pack $ show e)
                Right a ->
                    if a == zero
                    then throwError $
                        "So sorry, but user `" <> name <> "` have no account!"
                    else return (AccountAddress a)
