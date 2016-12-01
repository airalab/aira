-- |
-- Module      :  Aira.Contract.AiraEtherFunds
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  unknown
--
-- AiraEtherFunds contract API.
--
module Aira.Contract.AiraEtherFunds (
    secureUnapprove
  , secureApprove
  , transferFrom
  , getBalance
  , allowance
  , balanceOf
  , sendFrom
  , SHA3
) where

import Control.Monad.Error.Class (throwError)
import Crypto.Hash (hash, Digest, Keccak_256)
import Data.HexString (hexString, toBytes)
import Data.Text.Encoding (encodeUtf8)
import Data.Text (Text, pack, unpack)
import Network.Ethereum.Web3.Address
import Data.Text.Read (hexadecimal)
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Aira.Registrar

type SHA3 = Digest Keccak_256

transferFrom :: SHA3 -> SHA3 -> Double -> Web3 Text
transferFrom from dest amountFloat = do
    airaEtherFunds <- resolve "AiraEtherFunds.contract"
    self           <- resolve "AiraEth.bot"
    let amount = toWei amountFloat
        token_address = "0x" <> toText airaEtherFunds
        self_address  = "0x" <> toText self
        tranCall = Call (Just self_address) token_address Nothing Nothing Nothing . Just
        tranData = "0x2fb840f5" <> pack (show from)
                                <> pack (show dest)
                                <> paddedInt amount
     in eth_sendTransaction (tranCall tranData)

sendFrom :: SHA3 -> Address -> Double -> Web3 Text
sendFrom from dest amount = do
    airaEtherFunds <- resolve "AiraEtherFunds.contract"
    self           <- resolve "AiraEth.bot"
    let aira_address = "0x" <> toText airaEtherFunds
        self_address = "0x" <> toText self
        sendCall = Call (Just self_address) aira_address Nothing Nothing Nothing . Just
        sendData = "0x47d83127" <> pack (show from)
                                <> paddedAddr (toText dest)
                                <> paddedInt (toWei amount)
     in eth_sendTransaction (sendCall sendData)

-- | Allowed balance for bot
getBalance :: SHA3 -> Web3 Double
getBalance ident = do
    self    <- sha3Address <$> resolve "AiraEth.bot"
    balance <- balanceOf ident
    allow   <- ident `allowance` self
    return $
        if balance > allow then allow else balance
  where
    sha3Address = hash . toBytes . hexString . encodeUtf8 . toText

balanceOf :: SHA3 -> Web3 Double
balanceOf ident = do
    airaEtherFunds <- resolve "AiraEtherFunds.contract"
    let token_address = "0x" <> toText airaEtherFunds
        tokenCall = Call Nothing token_address Nothing Nothing Nothing . Just
        tokenData = "0x6c7f1542" <> pack (show ident)
    res <- eth_call (tokenCall tokenData) "latest"
    case hexadecimal res of
        Right (x, "") -> return (fromWei x)
        Right (_, e)  -> throwError (ParserFail $ unpack e)
        Left e        -> throwError (ParserFail e)

allowance :: SHA3 -> SHA3 -> Web3 Double
allowance from dest = do
    airaEtherFunds <- resolve "AiraEtherFunds.contract"
    let token_address = "0x" <> toText airaEtherFunds
        tokenCall = Call Nothing token_address Nothing Nothing Nothing . Just
        tokenData = "0xf2a9a8c7" <> pack (show from) <> pack (show dest)
    res <- eth_call (tokenCall tokenData) "latest"
    case hexadecimal res of
        Right (x, "") -> return (fromWei x)
        Right (_, e)  -> throwError (ParserFail $ unpack e)
        Left e        -> throwError (ParserFail e)

secureApprove :: SHA3 -> Double -> Web3 Text
secureApprove ident amount = do
    airaEtherFunds <- resolve "AiraEtherFunds.contract"
    self           <- resolve "AiraSecure.bot"
    let aira_address = "0x" <> toText airaEtherFunds
        self_address = "0x" <> toText self
        sendCall = Call (Just self_address) aira_address Nothing Nothing Nothing . Just
        sendData = "0xe09ca60c" <> pack (show ident)
                                <> paddedInt (toWei amount)
     in eth_sendTransaction (sendCall sendData)

secureUnapprove :: SHA3 -> Web3 Text
secureUnapprove ident = do
    airaEtherFunds <- resolve "AiraEtherFunds.contract"
    self           <- resolve "AiraSecure.bot"
    let aira_address = "0x" <> toText airaEtherFunds
        self_address = "0x" <> toText self
        sendCall = Call (Just self_address) aira_address Nothing Nothing Nothing . Just
        sendData = "0x366b6e9e" <> pack (show ident)
     in eth_sendTransaction (sendCall sendData)
