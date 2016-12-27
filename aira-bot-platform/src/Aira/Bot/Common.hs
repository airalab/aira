{-# LANGUAGE TypeSynonymInstances #-}
{-# LANGUAGE FlexibleInstances    #-}
{-# LANGUAGE DataKinds            #-}
-- |
-- Module      :  Aira.Bot.Common
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- Aira common bot stories.
--
module Aira.Bot.Common (
    etherscan_addr
  , etherscan_tx
  , addressHash
  , secure
  , about
  , start
  ) where

import qualified Data.ByteString.Base16 as B16
import Control.Monad.Error.Class (throwError)
import Crypto.Hash (hash, Digest, Keccak_256)
import Control.Monad.IO.Class (liftIO)
import Data.Text.Encoding (encodeUtf8)
import qualified Data.ByteArray as BA
import Network.Ethereum.Web3.Address
import Network.Ethereum.Web3
import Text.Read (readMaybe)
import Data.Monoid ((<>))
import Web.Telegram.Bot
import Data.Text as T

import qualified Aira.Contract.AiraEtherFunds as AEF
import Aira.Registrar
import Aira.Account

etherscan_tx :: Text -> Text
etherscan_tx tx = "[" <> tx <> "](https://etherscan.io/tx/" <> tx <> ")"

etherscan_addr :: Address -> Text
etherscan_addr a = "[" <> toText a <> "](https://etherscan.io/address/" <> toText a <> ")"

addressHash :: Address -> BytesN 32
addressHash = BytesN . BA.convert . keccak
  where keccak :: Address -> Digest Keccak_256
        keccak = hash . fst . B16.decode . encodeUtf8 . toText

instance Answer Address where
    parse msg = case text msg of
        Nothing -> throwError "Please send me Ethereum address."
        Just addr -> case fromText addr of
            Left e -> throwError (T.pack e)
            Right a -> return a

instance Answer Ether where
    parse msg = case (readMaybe . (++ " ether") . T.unpack) =<< text msg of
        Nothing -> throwError "Please send me a value in ether."
        Just x -> if x > 0 then return x
                           else throwError "Please send me a positive value."

secure :: Story
secure _ = return . toMessage $ T.unlines
-- TODO: Fill text description
    [ "@AiraSecureBot" ]

about :: AccountedStory
about a = return $ toMessage $ T.unlines $
    [ "Hello, " <> accountFullname a <> "!"
    , "Your account is " <>
        case accountState a of
            Verified -> "verified"
            _ -> "anonymous"
    , "Ethereum address is " <>
        case accountAddress a of
            Just address -> etherscan_addr address
            Nothing -> "unknown" ]

start :: AccountedStory
start a@(Account{ accountState = Unknown }) = do
    liftIO $ runWeb3 (accountSimpleReg a)
    about a
start a = about a
