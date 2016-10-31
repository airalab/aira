-- |
-- Module      :  Aira.Bot.Contract
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- Aira contract API.
--
module Aira.Bot.Contract (
    secureUnapprove
  , secureApprove
  , transferFrom
  , getBalance
  , sendFrom
  ) where

import Network.Ethereum.Web3.Address
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Data.Text (Text)
import Data.Text.Read
import Aira.Registrar

transferFrom :: Address -> Address -> Double -> Web3 Text
transferFrom from dest amount = do
    airaEtherFunds <- resolve "AiraEtherFunds.contract"
    self           <- resolve "AiraEth.bot"
    let aira_address = "0x" <> toText airaEtherFunds
        self_address = "0x" <> toText self
        tranCall = Call (Just self_address) aira_address Nothing Nothing Nothing . Just
        tranData = "0x23b872dd" <> paddedAddr (toText from)
                                <> paddedAddr (toText dest)
                                <> paddedInt (toWei amount)
     in eth_sendTransaction (tranCall tranData)

sendFrom :: Address -> Address -> Double -> Web3 Text
sendFrom from dest amount = do
    airaEtherFunds <- resolve "AiraEtherFunds.contract"
    self           <- resolve "AiraEth.bot"
    let aira_address = "0x" <> toText airaEtherFunds
        self_address = "0x" <> toText self
        sendCall = Call (Just self_address) aira_address Nothing Nothing Nothing . Just
        sendData = "0x5c004bcc" <> paddedAddr (toText from)
                                <> paddedAddr (toText dest)
                                <> paddedInt (toWei amount)
     in eth_sendTransaction (sendCall sendData)

getBalance :: Address -> Web3 Double
getBalance address = do
    airaEtherFunds <- resolve "AiraEtherFunds.contract"
    self           <- resolve "AiraEth.bot"
    let aira_address = "0x" <> toText airaEtherFunds
        self_address = "0x" <> toText self
        airaCall = Call (Just self_address) aira_address Nothing Nothing Nothing . Just
        airaData = "0xf8b2cb4f" <> paddedAddr (toText address)
    res <- eth_call (airaCall airaData) "latest"
    return $ case hexadecimal res of
        Right (x, _) -> fromWei x
        Left e       -> 0

secureApprove :: Address -> Double -> Web3 Text
secureApprove address amount = do
    airaEtherFunds <- resolve "AiraEtherFunds.contract"
    self           <- resolve "AiraSecure.bot"
    let aira_address = "0x" <> toText airaEtherFunds
        self_address = "0x" <> toText self
        sendCall = Call (Just self_address) aira_address Nothing Nothing Nothing . Just
        sendData = "0x07414002" <> paddedAddr (toText address)
                                <> paddedInt (toWei amount)
     in eth_sendTransaction (sendCall sendData)

secureUnapprove :: Address -> Web3 Text
secureUnapprove address = do
    airaEtherFunds <- resolve "AiraEtherFunds.contract"
    self           <- resolve "AiraSecure.bot"
    let aira_address = "0x" <> toText airaEtherFunds
        self_address = "0x" <> toText self
        sendCall = Call (Just self_address) aira_address Nothing Nothing Nothing . Just
        sendData = "0xdfefaf3f" <> paddedAddr (toText address)
     in eth_sendTransaction (sendCall sendData)
