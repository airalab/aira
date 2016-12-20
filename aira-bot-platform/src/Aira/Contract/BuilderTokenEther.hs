{-# LANGUAGE QuasiQuotes #-}
-- |
-- Module      :  Aira.Contract.BuilderTokenEther
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- BuilderTokenEther contract API.
--
module Aira.Contract.BuilderTokenEther where

import Network.Ethereum.Web3.TH
import Network.Ethereum.Web3
import Data.Text (Text)

[abiFrom|abi/builder_token_ether.json|]
