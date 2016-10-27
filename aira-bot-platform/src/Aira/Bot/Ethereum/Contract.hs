-- |
-- Module      :  Aira.Bot.Ethereum.Contract
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- Aira Ethereum bot contract API.
--
module Aira.Bot.Ethereum.Contract (
    transferFrom
  , getBalance
  , sendFrom
  ) where

import Network.Ethereum.Web3.Address
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Data.Text (Text)
import Data.Text.Read
import Aira.Registrar

owner :: Address -> Web3 Address
owner = undefined

transferFrom :: Address -> Address -> Double -> Web3 Text
transferFrom from dest amount = do
    airaEtherFunds <- resolve "contract-AiraEtherFunds"
    self           <- owner airaEtherFunds
    let aira_address = "0x" <> toText airaEtherFunds
        self_address = "0x" <> toText self
        tranCall = Call (Just self_address) aira_address Nothing Nothing Nothing . Just
        tranData = "0x6d2cb794" <> paddedAddr (toText from)
                                <> paddedAddr (toText dest)
                                <> paddedInt (toWei amount)
     in eth_sendTransaction (tranCall tranData)

sendFrom :: Address -> Address -> Double -> Web3 Text
sendFrom from dest amount = do
    airaEtherFunds <- resolve "contract-AiraEtherFunds"
    self           <- owner airaEtherFunds
    let aira_address = "0x" <> toText airaEtherFunds
        self_address = "0x" <> toText self
        sendCall = Call (Just self_address) aira_address Nothing Nothing Nothing . Just
        sendData = "0xe1efda6d" <> paddedAddr (toText from)
                                <> paddedAddr (toText dest)
                                <> paddedInt (toWei amount)
     in eth_sendTransaction (sendCall sendData)

getBalance :: Address -> Web3 Double
getBalance address = do aira <- resolve "contract-AiraEtherFunds"
                        res <- eth_call (airaCall aira airaData) "latest"
                        return $ case hexadecimal res of
                            Right (x, _) -> fromWei x
                            Left e       -> 0
  where airaCall a = Call Nothing ("0x" <> toText a) Nothing Nothing Nothing . Just
        airaData = "0x70a08231" <> paddedAddr (toText address)
