{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE DataKinds   #-}
-- |
-- Module      :  Aira.Registrar
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- Aira registrar service.
--
module Aira.Registrar (
    getAddress
  , getContent
  , regAddress
  , regContent
  , regRegistrar
  , removeRecord
  ) where

import Data.Text.Encoding (encodeUtf8, decodeUtf8)
import Network.Ethereum.Web3.Address
import qualified Data.Text as T
import qualified Data.ByteArray as BA
import Network.Ethereum.Web3.TH
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Data.Text (Text)

[abiFrom|abi/registrar.json|]

-- | Constant registrar contract address
root_registrar_address :: Address
-- Mainnet ROOT
root_registrar_address = "0x37C0f21f9dc15bE832d06a3c79ee45d16f9ed1d6"
-- Testnet ROOT
--root_registrar_address = "0x2d420359f7D51272D49fB74656491F7cBC9bE2c3"

type DomainName = Text

leafRegistrar :: DomainName -> Web3 (Address, DomainName)
leafRegistrar domain = go path root_registrar_address
  where (name : path)   = T.split (== '.') domain
        go [] reg       = return (reg, name)
        go (x : xs) reg = go xs =<< subRegistrar reg x

-- | Associate domain with address by @AiraRegistrar@ service
-- @NOTICE@ You sould be owner of leaf registrar contract.
regAddress :: DomainName -> Address -> Web3 Text
regAddress name address = do
    (registrar, host) <- leafRegistrar name
    setAddr registrar nopay host address

-- | Associate domain with content by @AiraRegistrar@ service
-- @NOTICE@ You sould be owner of leaf registrar contract.
regContent :: DomainName -> Text -> Web3 Text
regContent name cont = do
    (registrar, host) <- leafRegistrar name
    setContent registrar nopay host (BytesN $ BA.convert $ encodeUtf8 cont)

-- | Remove registrar record at all
-- @NOTICE@ You sould be owner of leaf registrar contract.
removeRecord :: DomainName -> Web3 Text
removeRecord name = do
    (registrar, host) <- leafRegistrar name
    disown registrar nopay host

-- | Associate name with registrar by @AiraRegistrar@ service
-- @NOTICE@ You sould be owner of leaf registrar contract.
regRegistrar :: DomainName -> Address -> Web3 Text
regRegistrar name address = do
    (registrar, host) <- leafRegistrar name
    setSubRegistrar registrar nopay host address

-- | Resolve name by address with @AiraRegistrar@ service
-- Sub registrar names is separated by dot symbol for tree
-- hierarchy naming.
getAddress :: DomainName -> Web3 Address
getAddress name = do
    (registrar, host) <- leafRegistrar name
    addr registrar host

-- | Take a content by given domain name with @AiraRegistrar@ service
getContent :: DomainName -> Web3 Text
getContent name = do
    (registrar, host) <- leafRegistrar name
    (decodeUtf8 . BA.convert . (unBytesN :: BytesN 32 -> BA.Bytes))
        <$> content registrar host
