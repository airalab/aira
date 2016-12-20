{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE DataKinds   #-}
-- |
-- Module      :  Aira.Contract.BuilderDeal
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- BuilderDeal contract API.
--
module Aira.Contract.BuilderDeal where

import Network.Ethereum.Web3.TH
import Network.Ethereum.Web3
import Data.Text (Text)

[abiFrom|abi/builder_deal.json|]
