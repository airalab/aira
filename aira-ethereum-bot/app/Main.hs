{-# LANGUAGE OverloadedLists #-}
module Main where

import Data.Acid (openLocalState)
import qualified Data.Text as T
import qualified Story as Story
import Data.Default.Class (def)
import Web.Telegram.Bot
import Data.Text (Text)
import ActivationCode
import Database

helpMessage :: Text
helpMessage = T.unlines
    [ "You can control me by sending these commands:"
    , ""
    , "/me - show information about your account"
    , "/send - send money to Ethereum account"
    , "/transfer - money transfer to Telegram account"
--    , "/event - append event listener"
--    , "/event_off - remove event listener"
    , "/delete - remove account address"
    , "/cancel - stop command execution"
    , "/help or any text - show this message" ]

main :: IO ()
main = do
    -- Open database
-- db <- openLocalState def
    codedb <- openLocalState def
    -- Run bot
    runBot config $ do
        activationCodeBot codedb
        storyBot helpMessage $
            [ ("/me", Story.about)
            , ("/start", Story.start codedb)
            , ("/send", Story.send)
            , ("/transfer", Story.transfer)
            , ("/delete", Story.delete)
--            , ("/event", Story.event db)
--            , ("/event_off", Story.eventOff db)
            ]
  where config = defaultConfig
            { token = Token "bot..." }
