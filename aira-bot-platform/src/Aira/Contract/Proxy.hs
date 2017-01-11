{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE DataKinds   #-}
-- |
-- Module      :  Aira.Contract.Proxy
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- Proxy contract API.
--
module Aira.Contract.Proxy where

import Network.Ethereum.Web3.TH
import Network.Ethereum.Web3

[abiFrom|abi/proxy.json|]
