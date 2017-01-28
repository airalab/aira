{-# LANGUAGE OverloadedLists #-}
module Main where

import qualified Aira.Bot.Factory as Factory
import qualified Aira.Bot.Common  as Common
import qualified Aira.Bot.Proxy   as Proxy
import qualified Aira.Bot.Token   as Token
import Aira.Account (accounting)
import qualified Data.Text as T
import Data.Text (Text)
import Web.Bot

helpMessage :: Text
helpMessage = T.unlines
    [ "You can control me by sending:"
    , ""
    , "/me - show information about your account"
    , "/send - send money to Ethereum account"
    , "/transfer - money transfer to Telegram account"
    , "/newtoken - create new token by Factory"
    , "/balance - get avail balance"
    , "/secure - get information about security bot"
    , "/refill - refill Air balance"
    , "/ident - get self user identity"
    , "/cancel - stop command execution"
    , "/help - show this message" ]

telegramBot :: Bot Telegram ()
telegramBot = do
    Proxy.proxyNotifyBot
    storyBot helpMessage $ fmap accounting
        [ ("/me",       Common.about)
        , ("/send",     Token.send)
        , ("/start",    Common.start)
        , ("/ident",    Common.ident)
        , ("/refill",   Token.refill)
        , ("/secure",   Common.secure)
        , ("/approve",  Token.approve)
        , ("/newtoken", Factory.createToken)
        , ("/balance",  Token.balance)
        , ("/transfer", Token.transfer)
        ]

main :: IO ()
main = runBot telegramBot
