{-# LANGUAGE OverloadedLists #-}
module Main where

import qualified Aira.Bot.Security.Story as Story
import qualified Aira.Bot.Story as CommonStory
import Aira.Bot.Security.Watch (listenBlocks)
import Data.Yaml (decodeFileEither)
import qualified Data.Text as T
import Data.Default.Class (def)
import Web.Telegram.Bot
import Data.Text (Text)
import Data.Acid

helpMessage :: Text
helpMessage = T.unlines
    [ "You can control me by sending these commands:"
    , ""
    , "/me - show information about your account"
    , "/approve - approve value for @AiraEthBot"
    , "/unapprove - cancel approve for @AiraEthBot"
    , "/watch - waiting for incoming transactions"
    , "/unwatch - drop incoming transactions listener"
    , "/cancel - stop command execution"
    , "/help or any text - show this message" ]

main :: IO ()
main = do
    -- Load config
    Right config <- decodeFileEither "config.yaml"
    -- Open database
    db <- openLocalState def
    -- Run bot
    runBot config $ do
        listenBlocks db
        storyBot helpMessage $
            [ ("/me", CommonStory.about)
            , ("/approve", Story.approve)
            , ("/unapprove", Story.unapprove)
            , ("/watch", Story.watch db)
            , ("/unwatch", Story.unwatch db)
            ]
