-- |
-- Module      :  Aira.TextFormat
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- Text formatting utils.
--
module Aira.TextFormat (
    uri_address
  , uri_block
  , uri_tx
  , Text
  , (<>)
  ) where

import Network.Ethereum.Web3.Address
import Data.Monoid ((<>))
import Data.Text

uri_tx :: Text -> Text
uri_tx tx = "[" <> tx <> "](https://etherscan.io/tx/" <> tx <> ")"
--uri_tx tx = "[" <> tx <> "](https://testnet.etherscan.io/tx/" <> tx <> ")"

uri_address :: Address -> Text
uri_address a = "[" <> toText a <> "](https://etherscan.io/address/" <> toText a <> ")"
--uri_address a = "[" <> toText a <> "](https://testnet.etherscan.io/address/" <> toText a <> ")"

uri_block :: Integer -> Text
uri_block b = "[" <> pack (show b) <> "](https://etherscan.io/block/" <> pack (show b) <> ")"
--uri_block b = "[" <> pack (show b) <> "](https://testnet.etherscan.io/block/" <> pack (show b) <> ")"
