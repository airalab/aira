{-# LANGUAGE QuasiQuotes #-}
-- |
-- Module      :  Aira.Contract.Token
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- ERC20 token contract API.
--
module Aira.Contract.Token where

import Network.Ethereum.Web3.TH
import Network.Ethereum.Web3

[abiFrom|abi/token.json|]

fromDecimals :: Address -> Integer -> Web3 Double
fromDecimals token raw = scale <$> decimals token
  where scale :: Int -> Double
        scale = (fromIntegral raw /) . (10^)

toDecimals :: Address -> Double -> Web3 Integer
toDecimals token scaled = scale <$> decimals token
  where scale :: Int -> Integer
        scale = round . (scaled *) . (10^)
