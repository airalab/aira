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
    etherscan_addr
  , etherscan_tx
  , Text
  , (<>)
  ) where

import Network.Ethereum.Web3.Address
import Data.Monoid ((<>))
import Data.Text

etherscan_tx :: Text -> Text
etherscan_tx tx = "[" <> tx <> "](https://etherscan.io/tx/" <> tx <> ")"

etherscan_addr :: Address -> Text
etherscan_addr a = "[" <> toText a <> "](https://etherscan.io/address/" <> toText a <> ")"
