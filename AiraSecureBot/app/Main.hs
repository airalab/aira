{-# LANGUAGE OverloadedLists #-}
module Main where

import qualified Aira.Bot.Secure as Secure
import qualified Aira.Bot.Common as Common
import Aira.Account (accounting)

import Aira.Bot.Watch (listenBlocks)
import Data.Yaml (decodeFileEither)
import qualified Data.Text as T
import Data.Default.Class (def)
import Web.Telegram.Bot
import Data.Text (Text)
import Data.Acid

helpMessage :: Text
helpMessage = T.unlines
    [ "You can control me by sending:"
    , ""
    , "/me - show information about your account"
    , "/approve - approve value for @AiraEthBot"
    , "/unapprove - cancel approve for @AiraEthBot"
    , "/watch - waiting for incoming transactions"
    , "/unwatch - drop incoming transactions listener"
    , "/cancel - stop command execution"
    , "/help - show this message" ]

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
    db <- openLocalState def
    -- Run bot
    runBot config $ do
        listenBlocks db
        storyBot helpMessage $
            [ ("/me",        accounting Common.about)
            , ("/start",     accounting Common.start)
            , ("/approve",   accounting Secure.approve)
            , ("/unapprove", accounting Secure.unapprove)
            , ("/watch",     accounting $ Secure.watch db)
            , ("/unwatch",   accounting $ Secure.unwatch db)
            ]
