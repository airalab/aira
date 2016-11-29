-- |
-- Module      :  Aira.Contract.Token
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- ERC20 token contract API.
--
module Aira.Contract.Token (
    transferFrom
  , balanceOf
  ) where

import Control.Monad.Error.Class (throwError)
import Network.Ethereum.Web3.Address
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Data.Text (Text)
import Data.Text.Read
import Aira.Registrar

decimals :: Address -> Web3 Int
decimals token = do
    res <- eth_call decimalsCall "latest"
    case hexadecimal res of
        Right (x, _) -> return x
        Left e       -> throwError (ParserFail e)
  where token_address = "0x" <> toText token
        decimalsCall  = Call Nothing token_address Nothing Nothing Nothing (Just "0x313ce567")

fromDecimals :: Address -> Integer -> Web3 Double
fromDecimals token raw = (fromIntegral raw /) . (10^) <$> decimals token

toDecimals :: Address -> Double -> Web3 Integer
toDecimals token scaled = round . (scaled *) . (10^) <$> decimals token

transferFrom :: Address -> Address -> Address -> Double -> Web3 Text
transferFrom token from dest amountFloat = do
    self <- resolve "AiraEth.bot"
    amount <- toDecimals token amountFloat
    let token_address = "0x" <> toText token
        self_address  = "0x" <> toText self
        tranCall = Call (Just self_address) token_address Nothing Nothing Nothing . Just
        tranData = "0x23b872dd" <> paddedAddr (toText from)
                                <> paddedAddr (toText dest)
                                <> paddedInt amount
     in eth_sendTransaction (tranCall tranData)

balanceOf :: Address -> Address -> Web3 Double
balanceOf token address = do
    let token_address = "0x" <> toText token
        tokenCall = Call Nothing token_address Nothing Nothing Nothing . Just
        tokenData = "0x70a08231" <> paddedAddr (toText address)
    res <- eth_call (tokenCall tokenData) "latest"
    case hexadecimal res of
        Right (x, _) -> fromDecimals token x
        Left e       -> throwError (ParserFail e)
