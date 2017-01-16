{-# LANGUAGE OverloadedLists #-}
module Main where

import qualified Aira.Bot.Factory as Factory
import qualified Aira.Bot.Common  as Common
import qualified Aira.Bot.Proxy   as Proxy
import qualified Aira.Bot.Token   as Token
import Aira.Account (accounting)
import Aira.Config (airaBot)

import Data.Acid (openLocalState)
import Data.Default.Class (def)
import qualified Data.Text as T
import Web.Telegram.Bot
import Data.Text (Text)

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
    , "/cancel - stop command execution"
    , "/help - show this message" ]

main :: IO ()
main = do
    -- Open database
    db <- openLocalState def
    -- Run bot
    airaBot $ do
        Proxy.proxyNotifyBot db
        storyBot helpMessage $ fmap (accounting db)
            [ ("/me",       Common.about)
            , ("/send",     Token.send)
            , ("/start",    Common.start)
            , ("/refill",   Token.refill)
            , ("/secure",   Common.secure)
            , ("/newtoken", Factory.createToken)
            , ("/balance",  Token.balance)
            , ("/transfer", Token.transfer)
            ]
