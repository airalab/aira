{-# LANGUAGE FlexibleContexts #-}
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
  , floatToText
  , addressHash
  , secure
  , about
  , start
  ) where

import Control.Monad.Error.Class (throwError)
import Data.Text.Lazy.Builder (toLazyText)
import Data.HexString (hexString, toBytes)
import Data.Text.Lazy.Builder.RealFloat
import Control.Monad.IO.Class (liftIO)
import Data.Text.Encoding (encodeUtf8)
import Network.Ethereum.Web3.Address
import Control.Applicative ((<|>))
import Data.Text.Lazy (toStrict)
import Network.Ethereum.Web3
import Crypto.Hash (hash)
import Data.Monoid ((<>))
import Web.Telegram.Bot
import Data.Text as T

import qualified Aira.Contract.AiraEtherFunds as AEF
import Aira.Registrar
import Aira.Account

etherscan_tx :: Text -> Text
etherscan_tx tx = "[" <> tx <> "](https://etherscan.io/tx/" <> tx <> ")"

etherscan_addr :: Text -> Text
etherscan_addr a = "[" <> a <> "](https://etherscan.io/address/" <> a <> ")"

floatToText :: RealFloat a => a -> Text
floatToText = toStrict . toLazyText . formatRealFloat Fixed Nothing

addressHash :: Address -> AEF.SHA3
addressHash = hash . toBytes . hexString . encodeUtf8 . toText

instance Answer Address where
    parse msg = case text msg of
        Nothing -> throwError "Please send me Ethereum address."
        Just addr -> case fromText addr of
            Left e -> throwError (T.pack e)
            Right a -> return a

secure :: Story
secure _ = return . toMessage $ T.unlines
-- TODO: Fill text description
    [ "@AiraSecureBot" ]

about :: AccountedStory
about a = return $ toMessage $ T.unlines $
    [ "Hello, " <> accountFullname a <> "!"
    , "Account: " <> T.pack (show $ accountState a)
    , "Your address: " <>
        case accountAddress a of
            Just address -> etherscan_addr (toText address)
            Nothing -> "None"
    , "Do you want to check /balance?" ]

start :: AccountedStory
start a@(Account{ accountState = Unknown }) = do
    liftIO $ runWeb3 (accountSimpleReg a)
    about a
start a = about a
