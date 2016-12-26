{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE TemplateHaskell  #-}
{-# LANGUAGE DataKinds        #-}
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
  , accountSimpleReg
  , AccountedStory
  , accountDelete
  , accountVerify
  , loadAccount
  , accounting
  ) where

import Web.Telegram.API.Bot (Chat(..), ChatType(..))
import Web.Telegram.API.Bot.Data (Message(text))
import Control.Monad.Error.Class (throwError)
import Crypto.Hash (hash, Digest, Keccak_256)
import Network.Ethereum.Web3.Address (zero)
import Control.Monad.IO.Class (liftIO)
import Data.Text.Encoding (encodeUtf8)
import qualified Data.ByteArray as BA
import Control.Applicative ((<|>))
import qualified Data.Text as T
import Web.Telegram.Bot.Story
import Web.Telegram.Bot.Types
import Network.Ethereum.Web3
import Text.Read (readMaybe)
import Control.Monad (when)
import Data.Monoid ((<>))
import Data.Text (Text)
import Aira.Registrar
import Data.SafeCopy

data AccountState = Unknown
                  | Verified
                  | Unverified
  deriving (Show, Eq, Read)

instance Ord Chat where
    compare a b = compare (chat_id a) (chat_id b)

instance Eq Chat where
    a == b = chat_id a == chat_id b

$(deriveSafeCopy 0 'base ''Chat)
$(deriveSafeCopy 0 'base ''ChatType)

data Account
  = Account
  { accountChat     :: Chat
  , accountFullname :: Text
  , accountUsername :: Text
  , accountHash     :: BytesN 32
  , accountAddress  :: Maybe Address
  , accountState    :: AccountState
  } deriving (Show, Eq)

type AccountedStory = Account -> StoryT Bot BotMessage

instance Answer Account where
    parse msg = case text msg of
        Nothing -> throwError "Please send me text username."
        Just name -> do
            let name' = T.replace "!" "" (T.replace "@" "" name)
                pChat = Chat 0 Private Nothing (Just name') Nothing Nothing
                force = T.takeEnd 1 name == "!"
            res <- liftIO $ runWeb3 (loadAccount pChat)
            case res of
                Left e  -> throwError (T.pack $ show e)
                Right a -> case (accountState a, force) of
                    (Unknown, False) -> throwError $ T.unlines $
                        [ "I don't known `" <> name <> "`!"
                        , "You can specify yet by appending '!' char to his name."
                        , "Of course at your peril."
                        , "Example: `" <> name <> "!`"]
                    _ -> return a

newtype AccountAddress = AccountAddress Address
  deriving (Show, Eq)

instance Answer AccountAddress where
    parse msg = do
        acc <- parse msg
        case accountAddress acc of
            Nothing -> throwError $ "So sorry, but user `"
                                  <> accountUsername acc
                                  <> "` have no linked Ethereum address!"
            Just a -> return (AccountAddress a)

withUsername :: Story -> Story
withUsername story c =
    case chat_username c of
        Just _ -> story c
        Nothing -> return $ toMessage $ T.unlines
            [ "Hi! You don't use Telegram username!"
            , "But it's required for Aira identity systems."
            , "Please go to Telegram app settings and"
            , "fill `username` field." ]

getName :: Chat -> (Text, Text)
getName c = (full, user)
  where Just full = chat_first_name c <|> chat_username c
        Just user = T.toLower <$> chat_username c

-- | Take address by account name
loadAccount :: Chat -> Web3 Account
loadAccount c = Account c full user ident
    <$> (fmap zeroMaybe (getAddress (identText <> ".account")))
    <*> (fmap readAccSt (getContent (identText <> ".account")))
  where (full, user) = getName c
        keccak :: Digest Keccak_256
        keccak = hash (encodeUtf8 user)
        ident :: BytesN 32
        ident  = BytesN (BA.convert keccak)
        identText = T.pack (show ident)
        zeroMaybe a | a == zero = Nothing
                    | otherwise = Just a
        readAccSt t = case readMaybe (T.unpack t) of
            Nothing -> Unknown
            Just s  -> s

-- | Full verification of account
accountVerify :: Account -> Address -> Web3 Text
accountVerify acc addr = do
    regAddress (ident acc <> ".account") addr
    regContent (ident acc <> ".account") (T.pack $ show Verified)
  where ident = T.pack . show . accountHash

-- Take name and remove record from registrar
accountDelete :: Account -> Web3 Text
accountDelete acc = removeRecord (ident acc <> ".account")
  where ident = T.pack . show . accountHash

-- User accounting combinator
accounting :: AccountedStory -> Story
accounting story = withUsername $ \c -> do
    res <- liftIO $ runWeb3 (loadAccount c)
    case res of
        Left e  -> return $ toMessage (T.pack $ show e)
        Right a -> story a

-- | Simple account registration
accountSimpleReg :: Account -> Web3 Text
accountSimpleReg acc =
    regContent (ident acc <> ".account")
               (T.pack $ show Unverified)
  where ident = T.pack . show . accountHash
