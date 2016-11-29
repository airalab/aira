{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE TemplateHaskell  #-}
-- |
-- Module      :  Aira.Account
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  unknown
--
-- AIRA accounting system.
--
module Aira.Account (
    Account(..)
  , AccountAddress(..)
  , AccountState(..)
  , AccountedStory
  , accountDelete
  , accountVerify
  , loadAccount
  , accounting
  ) where

import Web.Telegram.API.Bot (Chat(..), ChatType(..))
import Web.Telegram.API.Bot.Data (Message(text))
import Control.Monad.Error.Class (throwError)
import Aira.Contract.AiraEtherFunds (SHA3)
import Control.Monad.IO.Class (liftIO)
import Data.Text.Encoding (encodeUtf8)
import Network.Ethereum.Web3.Address
import Control.Applicative ((<|>))
import qualified Data.Text as T
import Web.Telegram.Bot.Story
import Web.Telegram.Bot.Types
import Network.Ethereum.Web3
import Text.Read (readMaybe)
import Control.Monad (when)
import Crypto.Hash (hash)
import Data.Monoid ((<>))
import Data.Text (Text)
import Aira.Registrar
import Data.SafeCopy

data AccountState = Unknown
                  | Verified
                  | Unverified
  deriving (Show, Eq, Read)

$(deriveSafeCopy 0 'base ''Chat)
$(deriveSafeCopy 0 'base ''ChatType)

instance Ord Chat where
    compare a b = compare (chat_id a) (chat_id b)

instance Eq Chat where
    a == b = chat_id a == chat_id b

data Account
  = Account
  { accountChat     :: Chat
  , accountFullname :: Text
  , accountUsername :: Text
  , accountHash     :: SHA3
  , accountAddress  :: Maybe Address
  , accountState    :: AccountState
  } deriving (Show, Eq)

type AccountedStory = Account -> StoryT Bot BotMessage

newtype AccountAddress = AccountAddress Address
  deriving (Show, Eq)

instance Answer AccountAddress where
    parse msg = case text msg of
        Nothing -> throwError "Please send me text username."
        Just name -> liftIO (runWeb3 $ resolve (T.toLower name <> ".account"))
                     >>= go name
      where go name (Right address)
                | address == zero
                    = throwError $ "User `" <> name <> "` have no address!"
                | otherwise
                    = return (AccountAddress address)
            go _ (Left e) = throwError (T.pack $ show e)

instance Answer Account where
    parse msg = case text msg of
        Nothing -> throwError "Please send me text username."
        Just name -> do
            res <- liftIO $ runWeb3 $
                loadAccount (Chat 0 Private Nothing (Just name) Nothing Nothing)
            case res of
                Right a -> return a
                Left e  -> throwError (T.pack $ show e)

withUsername :: Story -> Story
withUsername story c =
    case chat_username c of
        Just _ -> story c
        Nothing -> return $ toMessage $ T.unlines
            [ "Hi! You have not use Telegram username!"
            , "It's required for Aira identity systems."
            , "Please go to Telegram app settings and"
            , "fill `username` field." ]

getName :: Chat -> (Text, Text)
getName c = (full, user)
  where Just full = chat_first_name c <|> chat_username c
        Just user = T.toLower <$> chat_username c

-- | Take address by account name
loadAccount :: Chat -> Web3 Account
loadAccount c = Account c full user (hash $ encodeUtf8 user)
    <$> (fmap zeroMaybe (resolve (user <> ".account")))
    <*> (fmap readAccSt (content (user <> ".account")))
  where (full, user) = getName c
        zeroMaybe a | a == zero = Nothing
                    | otherwise = Just a
        readAccSt t = case readMaybe (T.unpack t) of
            Nothing -> Unknown
            Just s  -> s

-- | Full verification of account
accountVerify :: Text -> Address -> Web3 Text
accountVerify user addr = do
    setAddress (user <> ".account") addr
    setContent (user <> ".account") (T.pack $ show Verified)

-- Take name and remove record from registrar
accountDelete :: Text -> Web3 Text
accountDelete user = do
    setAddress (user <> ".account") zero
    setContent (user <> ".account") ""

-- User accounting combinator
accounting :: AccountedStory -> Story
accounting story = withUsername $ \c -> do
    res <- liftIO $ runWeb3 (loadAccount c)
    case res of
        Left e -> return $ toMessage (T.pack $ show e)
        Right a -> do
            when (accountState a == Unknown) $ do
                liftIO $ runWeb3 $ accountSimpleReg (accountUsername a)
                return ()
            story $ a {accountState = Unverified}

-- | Simple account registration
accountSimpleReg :: Text -> Web3 Text
accountSimpleReg user =
    setContent (user <> ".account") (T.pack $ show Unverified)
