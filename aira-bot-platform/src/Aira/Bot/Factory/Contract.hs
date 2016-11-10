-- |
-- Module      :  Aira.Bot.Factory.Contract
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- Aira contract API.
--
module Aira.Bot.Factory.Contract (
    createToken
  ) where

import Network.Ethereum.Web3.Address
import qualified Data.Text as T
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Data.Text (Text)
import Data.Text.Read
import Aira.Registrar

createToken :: Text
            -> Address
            -> Text
            -> Text
            -> Int
            -> Integer
            -> Web3 Text
createToken contract client name symbol decimals total = do
    builder <- resolve contract
    self    <- resolve "AiraEth.bot"
    let builder_address = "0x" <> toText builder
        self_address    = "0x" <> toText self
        createCall = Call (Just self_address) builder_address Nothing Nothing Nothing . Just
        encodedName     = paddedText name
        encodedSymbol   = paddedText symbol
        createData = "0x106c5511" <> paddedAddr (toText client)
                                  <> paddedInt 160
                                  <> paddedInt (160 + (T.length encodedName `div` 2))
                                  <> paddedInt decimals
                                  <> paddedInt total
                                  <> encodedName
                                  <> encodedSymbol
      in eth_sendTransaction (createCall createData)
