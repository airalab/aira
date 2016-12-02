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
  , secure
  , about
  , start
  ) where

import Control.Monad.Error.Class (throwError)
import Data.Text.Lazy.Builder (toLazyText)
import Data.Text.Lazy.Builder.RealFloat
import Control.Monad.IO.Class (liftIO)
import Network.Ethereum.Web3.Address
import Data.Text.Read (hexadecimal)
import Control.Applicative ((<|>))
import Data.Text.Lazy (toStrict)
import Network.Ethereum.Web3
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

instance Answer Address where
    parse msg = case text msg of
        Nothing -> throwError "Please send me Ethereum address."
        Just addr -> case fromText addr of
            Left e -> throwError (T.pack e)
            Right a -> return a

ethBalance :: Address -> Web3 Double
ethBalance address = do
    res <- eth_getBalance ("0x" <> toText address) "latest"
    case hexadecimal res of
        Right (x, _) -> return (fromWei x)
        Left e       -> throwError (ParserFail e)

secure :: Story
secure _ = return . toMessage $ T.unlines
-- TODO: Fill text description
    [ "@AiraSecureBot" ]

about :: AccountedStory
about a = do
    let info = (,,) <$> AEF.getBalance  (accountHash a)
                    <*> AEF.balanceOf   (accountHash a)
                    <*> mapM ethBalance (accountAddress a)
    res <- liftIO (runWeb3 info)
    return $ case res of
        Right (x, y, z) -> toMessage $ T.unlines
            [ "Hello, " <> accountFullname a <> "!"
            , "Account: " <> T.pack (show $ accountState a)
            , "Your address: " <> case accountAddress a of
                                    Just address -> etherscan_addr (toText address)
                                    Nothing -> "None"
            , "Aira balance: " <> floatToText x <> " `ETH` approved / "
                               <> floatToText y <> " `ETH` on contract"
                               <> case z of
                                    Just z' -> " / " <> floatToText z' <> " `ETH` on account"
                                    Nothing -> ""
            ]
        Left e -> toMessage $ pack (show e)

start :: AccountedStory
start a@(Account{ accountState = Unknown }) = do
    liftIO $ runWeb3 (accountSimpleReg a)
    about a
start a = about a
