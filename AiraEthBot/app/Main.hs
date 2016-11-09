{-# LANGUAGE OverloadedLists #-}
module Main where

import qualified Aira.Bot.Ethereum.Story as Story
import qualified Aira.Bot.Story as CommonStory
import Aira.Bot.Activation (listenCode)
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
    , "/balance - get avail balance"
    , "/secure - get information about security bot"
    , "/unregister - remove account address"
    , "/cancel - stop command execution"
    , "/help or any text - show this message" ]

main :: IO ()
main = do
    -- Load config
    Right config <- decodeFileEither "config.yaml"
    -- Open database
    codedb <- openLocalState def
    -- Run bot
    runBot config $ do
        listenCode codedb
        storyBot helpMessage $
            [ ("/me", CommonStory.about)
            , ("/start", CommonStory.start codedb)
            , ("/send", Story.send)
            , ("/secure", Story.secure)
            , ("/balance", Story.balance)
            , ("/transfer", Story.transfer)
            , ("/unregister", CommonStory.unregister)
            ]
