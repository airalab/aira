{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE DataKinds   #-}
-- |
-- Module      :  Aira.Contract.BuilderProxy
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- BuilderProxy contract API.
--
module Aira.Contract.BuilderProxy where

import Network.Ethereum.Web3.TH
import Network.Ethereum.Web3
import Data.Text (Text)

[abiFrom|abi/builder_proxy.json|]
