{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE DataKinds   #-}
-- |
-- Module      :  Aira.Contract.BuilderInvoice
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- BuilderInvoice contract API.
--
module Aira.Contract.BuilderInvoice where

import Network.Ethereum.Web3.TH
import Network.Ethereum.Web3
import Data.Text (Text)

[abiFrom|abi/builder_invoice.json|]
