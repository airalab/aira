{-# LANGUAGE OverloadedLists #-}
module Main where

import qualified Aira.Bot.Factory as Factory
import qualified Aira.Bot.Secure  as Secure
import qualified Aira.Bot.Common  as Common
import qualified Aira.Bot.Proxy   as Proxy
import qualified Aira.Bot.Token   as Token
import qualified Aira.Bot.Watch   as Watch
import Aira.Account (accounting)
import qualified Data.Text as T
import Data.Text (Text)
import Web.Bot

helpMessage :: Text
helpMessage = T.unlines
    [ "You can control me by sending:"
    , ""
    , "/me - show information about your account"
    , "/id - get self user identity number"
    , "/send - send money to Ethereum account"
    , "/transfer - money transfer to Telegram account"
    , "/newtoken - create new token by Factory"
    , "/balance - get avail balance"
    , "/watch - subscribe to account transactions"
    , "/unwatch - unsubscribe to account transactions"
    , "/cancel - stop command execution"
    , "/help - show this message" ]

telegramBot :: Bot Telegram ()
telegramBot = do
    forkBot Watch.blockListenerBot
    Proxy.proxyNotifyBot
    storyBot helpMessage $ fmap accounting
        [ ("/me",       Common.about)
        , ("/id",       Common.ident)
        , ("/send",     Token.send)
        , ("/start",    Common.start)
        , ("/watch",    Secure.watch)
        , ("/unwatch",  Secure.unwatch)
        , ("/balance",  Token.balance)
        , ("/approve",  Token.approve)
        , ("/transfer", Token.transfer)
        , ("/newtoken", Factory.createToken)
        ]

main :: IO ()
main = runBot telegramBot
