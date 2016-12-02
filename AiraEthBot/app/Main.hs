{-# LANGUAGE OverloadedLists #-}
module Main where

import qualified Aira.Bot.Activation as Activation
import qualified Aira.Bot.Factory    as Factory
import qualified Aira.Bot.Common     as Common
import qualified Aira.Bot.Token      as Token
import Aira.Account (accounting)

import Data.Yaml (decodeFileEither)
import Data.Acid (openLocalState)
import Data.Default.Class (def)
import qualified Data.Text as T
import Web.Telegram.Bot
import Data.Text (Text)

helpMessage :: Text
helpMessage = T.unlines
    [ "You can control me by sending these commands:"
    , ""
    , "/me - show information about your account"
    , "/send - send money to Ethereum account"
    , "/transfer - money transfer to Telegram account"
    , "/create - create new contract by Factory"
    , "/balance - get avail balance"
    , "/secure - get information about security bot"
    , "/unregister - remove account address"
    , "/cancel - stop command execution"
    , "/help or any text - show this message" ]

withConfig :: (Config -> IO ()) -> IO ()
withConfig f = do
    -- Load config
    res <- decodeFileEither "config.yaml"
    case res of
        Left e -> putStrLn (show e)
        Right config -> f config

main :: IO ()
main = withConfig $ \config -> do
    -- Open database
    codedb <- openLocalState def
    -- Run bot
    runBot config $ do
        Activation.listenCode codedb
        storyBot helpMessage $
            [ ("/me",         accounting Common.about)
            , ("/send",       accounting Token.send)
            , ("/start",      accounting Common.start)
            , ("/verify",     accounting $ Activation.verify codedb)
            , ("/secure",     Common.secure)
            , ("/create",     accounting Factory.create)
            , ("/balance",    accounting Token.balance)
            , ("/transfer",   accounting Token.transfer)
            , ("/unregister", accounting Activation.unregister)
            ]
