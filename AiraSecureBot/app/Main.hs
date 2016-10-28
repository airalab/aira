{-# LANGUAGE OverloadedLists #-}
module Main where

import qualified Aira.Bot.Security.Story as Story
import qualified Aira.Bot.Story as CommonStory
import qualified Data.Text as T
import Web.Telegram.Bot
import Data.Text (Text)

helpMessage :: Text
helpMessage = T.unlines
    [ "You can control me by sending these commands:"
    , ""
    , "/me - show information about your account"
    , "/approve - approve value for @AiraEthBot"
    , "/unapprove - cancel approve for @AiraEthBot"
--    , "/watch - remove account address"
    , "/cancel - stop command execution"
    , "/help or any text - show this message" ]

main :: IO ()
main = do
    -- Run bot
    runBot config $ do
        storyBot helpMessage $
            [ ("/me", CommonStory.about)
            , ("/approve", Story.approve)
            , ("/unapprove", Story.unapprove)
            ]
  where config = defaultConfig
            { token = Token "bot..." }
