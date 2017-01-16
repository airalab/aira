{-# LANGUAGE QuasiQuotes #-}
-- |
-- Module      :  Aira.Contract.TokenSelling
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- TokenSaler contract API.
--
module Aira.Contract.TokenSelling where

import Network.Ethereum.Web3.TH
import Network.Ethereum.Web3

[abiFrom|abi/token_selling.json|]
