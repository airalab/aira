{-# LANGUAGE QuasiQuotes #-}
-- |
-- Module      :  Aira.Contract.BuilderTokenEmission
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- BuilderTokenEmission contract API.
--
module Aira.Contract.BuilderTokenEmission where

import Network.Ethereum.Web3.TH
import Network.Ethereum.Web3
import Data.Text (Text)

[abiFrom|abi/builder_token_emission.json|]
