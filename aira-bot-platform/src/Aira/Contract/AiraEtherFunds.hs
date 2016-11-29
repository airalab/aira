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
  , balanceOf
  , sendFrom
  , SHA3
) where

import Control.Monad.Error.Class (throwError)
import Crypto.Hash (Digest, Keccak_256)
import Data.Text (Text, pack, unpack)
import Network.Ethereum.Web3.Address
import Data.Text.Read (hexadecimal)
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Aira.Registrar

type SHA3 = Digest Keccak_256

transferFrom :: SHA3 -> SHA3 -> Double -> Web3 Text
transferFrom = undefined

sendFrom :: SHA3 -> Address -> Double -> Web3 Text
sendFrom from dest amount = do
    airaEtherFunds <- resolve "AiraEtherFunds.contract"
    self           <- resolve "AiraEth.bot"
    let aira_address = "0x" <> toText airaEtherFunds
        self_address = "0x" <> toText self
        sendCall = Call (Just self_address) aira_address Nothing Nothing Nothing . Just
        sendData = "" <> pack (show from)
                                <> toText dest
                                <> paddedInt (toWei amount)
     in eth_sendTransaction (sendCall sendData)

-- | Allowed balance for bot
getBalance :: SHA3 -> Web3 Double
getBalance = undefined

balanceOf :: SHA3 -> Web3 Double
balanceOf ident = do
    airaEtherFunds <- resolve "AiraEtherFunds.contract"
    let token_address = "0x" <> toText airaEtherFunds
        tokenCall = Call Nothing token_address Nothing Nothing Nothing . Just
        tokenData = "" <> pack (show ident)
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
        sendData = "" <> pack (show ident)
                                <> paddedInt (toWei amount)
     in eth_sendTransaction (sendCall sendData)

secureUnapprove :: SHA3 -> Web3 Text
secureUnapprove ident = do
    airaEtherFunds <- resolve "AiraEtherFunds.contract"
    self           <- resolve "AiraSecure.bot"
    let aira_address = "0x" <> toText airaEtherFunds
        self_address = "0x" <> toText self
        sendCall = Call (Just self_address) aira_address Nothing Nothing Nothing . Just
        sendData = "" <> pack (show ident)
     in eth_sendTransaction (sendCall sendData)
