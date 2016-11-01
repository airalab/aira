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
    resolve
  , setAddress
  , setRegistrar
  ) where

import Control.Monad.Error.Class (throwError)
import Network.Ethereum.Web3.Address
import qualified Data.Text as T
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Data.Text (Text)

-- | Constant registrar contract address
root_registrar_address :: Address
root_registrar_address = address
  where Right address = fromText "0x1942fAB8D465a9aDd99962a7e8e93b50f7FCa2D3"

type DomainName = Text

leafRegistrar :: DomainName -> Web3 (Address, DomainName)
leafRegistrar domain = go path root_registrar_address
  where (name : path)   = T.split (== '.') domain
        go [] reg       = return (reg, name)
        go (x : xs) reg = do res <- eth_call (regCall reg (regData x)) "latest"
                             case fromText (T.drop 26 res) of
                                Right address -> go xs address
                                Left e -> throwError $
                                    UserFail ("Internal error: " ++ e)
        regCall reg   = Call Nothing ("0x" <> toText reg) Nothing Nothing Nothing . Just
        regData txt   = "0x7f445c24" <> paddedInt 32 <> text2data txt

-- | Associate domain with address by @AiraRegistrar@ service
-- @NOTICE@ You sould be owner of leaf registrar contract.
setAddress :: DomainName -> Address -> Web3 Text
setAddress name address = do
    (registrar, host) <- leafRegistrar name
    owner <- T.drop 26 <$> eth_call (ownerCall registrar) "latest"
    eth_sendTransaction $ regCall registrar owner (regData host)
  where regCall r o = Call (Just $ "0x" <> o) ("0x" <> toText r) Nothing Nothing Nothing . Just
        regData n   = "0x213b9eb8" <> paddedInt 64 <> paddedAddr (toText address) <> text2data n
        ownerCall r = Call Nothing ("0x" <> toText r) Nothing Nothing Nothing (Just "0x8da5cb5b")

-- | Associate name with registrar by @AiraRegistrar@ service
-- @NOTICE@ You sould be owner of leaf registrar contract.
setRegistrar :: DomainName -> Address -> Web3 Text
setRegistrar name address = do
    (registrar, host) <- leafRegistrar name
    owner <- T.drop 26 <$> eth_call (ownerCall registrar) "latest"
    eth_sendTransaction $ regCall registrar owner (regData host)
  where regCall r o = Call (Just $ "0x" <> o) ("0x" <> toText r) Nothing Nothing Nothing . Just
        regData n   = "0xccf4f413" <> paddedInt 64 <> paddedAddr (toText address) <> text2data n
        ownerCall r = Call Nothing ("0x" <> toText r) Nothing Nothing Nothing (Just "0x8da5cb5b")

-- | Resolve name by address with @AiraRegistrar@ service
-- Sub registrar names is separated by dot symbol for tree
-- hierarchy naming.
resolve :: DomainName -> Web3 Address
resolve name = do (registrar, host) <- leafRegistrar name
                  res <- eth_call (regCall registrar $ regData host) "latest"
                  case fromText (T.drop 26 res) of
                      Right a -> return a
                      Left e  -> throwError (UserFail $ "Internal error: " ++ e)
  where regCall r = Call Nothing ("0x" <> toText r) Nothing Nothing Nothing . Just
        regData n = "0x511b1df9" <> paddedInt 32 <> text2data n
