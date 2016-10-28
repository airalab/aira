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
module Aira.Registrar (resolve, setAddress) where

import Network.Ethereum.Web3.Address
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Data.Text (Text)
import Data.Text as T

-- | Constant registrar contract address
registrar_address :: Text
registrar_address = "0x7A5dA0C039e718F0cFea12B0Bb210A5Ed6219921"

-- | Register new address with name by @AiraRegistrar@ service
setAddress :: Text -> Address -> Web3 Text
setAddress name address = do
    owner <- eth_call ownerCall "latest"
    eth_sendTransaction $ regCall owner (Just regData)
  where regCall o = Call (Just o) registrar_address Nothing Nothing Nothing
        regData   = "0x213b9eb8" <> paddedInt 64 <> paddedAddr (toText address) <> text2data name
        ownerCall = Call Nothing registrar_address Nothing Nothing Nothing (Just "0x8da5cb5b")

-- | Resolve name by address with @AiraRegistrar@ service
resolve :: Text -> Web3 Address
resolve name = do res <- eth_call (regCall regData) "latest"
                  return $ case fromText (T.drop 26 res) of
                      Right a -> a
                      Left e  -> error (show e)
  where regCall = Call Nothing registrar_address Nothing Nothing Nothing . Just
        regData = "0x511b1df9" <> paddedInt 32 <> text2data name
