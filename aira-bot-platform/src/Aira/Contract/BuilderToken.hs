{-# LANGUAGE QuasiQuotes #-}
-- |
-- Module      :  Aira.Contract.BuilderToken
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- BuilderToken contract API.
--
module Aira.Contract.BuilderToken where

import Network.Ethereum.Web3.TH
import Network.Ethereum.Web3
import Data.Text (Text)

[abiFrom|abi/builder_token.json|]
