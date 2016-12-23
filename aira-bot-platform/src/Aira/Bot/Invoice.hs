-- |
-- Module      :  Aira.Bot.Invoice
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- Invoice stories.
--
module Aira.Bot.Invoice where

import qualified Aira.Contract.Invoice as Invoice
import qualified Data.Text             as T
import Network.Ethereum.Web3.Address
import Aira.Bot.Token (ethBalance)
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Web.Telegram.Bot
import Data.Text (Text)
import Aira.Bot.Common
import Aira.Account
import Pipes (yield)

invoice :: AccountedStory
invoice _ = do
    invoice <- question "Invoice address:"
    res <- runWeb3 $ do
        desc <- Invoice.description invoice
        sign <- Invoice.signer      invoice
        val  <- fromWei <$> Invoice.value invoice
        bal  <- ethBalance invoice
        return (bal == val, T.unlines [
            "Invoice at " <> etherscan_addr invoice
          , "\tBalance: " <> T.pack (show (bal :: Ether))
          , "\tSigner: " <> if sign == zero then "unsigned"
                                            else etherscan_addr sign
          , "\tValue: " <> T.pack (show val)
          , "\tDescription:"
          , desc
          ])
    case res of
        Left e -> return (toMessage (T.pack $ show e))
        Right (ready, info) -> do
            yield (toMessage info)
            if ready then do
                ans <- select "Invoice ready for withdraw, do you want?" [["Yes", "No"]]
                case ans :: Text of
                    "No" -> return $ toMessage ("Let me known if you change your mind." :: Text)
                    "Yes" -> do
                        res <- runWeb3 (Invoice.withdraw invoice nopay)
                        return $ toMessage $ case res of
                            Right tx -> "Success " <> etherscan_tx tx
                            Left e -> T.pack (show e)
            else return $ toMessage ("Invoice is waiting for payment." :: Text)
