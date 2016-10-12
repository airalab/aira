{-# LANGUAGE OverloadedLists #-}
module Main where

import Data.Default.Class (def)
import qualified Data.Text as T
import qualified Story as Story
import Data.Text (Text)
import Web.Telegram.Bot
import Data.Acid
import Database

helpMessage :: Text
helpMessage = T.unlines
    [ "You can control me by sending these commands:"
    , ""
    , "/me - show information about your account"
    , "/send - send money to another account"
    , "/event - append event listener"
    , "/event_off - remove event listener"
    , "/cancel - stop command execution"
    , "/help or any text - show this message" ]

main :: IO ()
main = do
    -- Open database
    db <- openLocalState def
    -- Run bot
    runBot config $ do
        storyBot helpMessage $
            [ ("/me", Story.about db)
            , ("/send", Story.send db)
            , ("/event", Story.event db)
            , ("/event_off", Story.eventOff db)
            ]
  where config = defaultConfig
            { token = Token "bot..." }
