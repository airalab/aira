{-# LANGUAGE DataKinds #-}
-- |
-- Module      :  Aira.Bot.Factory
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- Aira Ethereum bot stories.
--
module Aira.Bot.Factory (
    createInvoice
  , createToken
  ) where

import Control.Monad.IO.Class (liftIO)
import Control.Concurrent.Chan
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Web.Telegram.Bot
import Data.Text as T
import Pipes (yield)

import qualified Aira.Contract.Invoice              as Invoice
import qualified Aira.Contract.BuilderInvoice       as BInvoice
import qualified Aira.Contract.BuilderToken         as BToken
import qualified Aira.Contract.BuilderTokenEther    as BTokenEther
import qualified Aira.Contract.BuilderTokenEmission as BTokenEmission
import Aira.Bot.Common
import Aira.Registrar
import Aira.Account

-- | Create Invoice contract by Factory
createInvoice :: AccountedStory
createInvoice (Account{accountHash = ident}) = do
    desc <- question "Description:"
    amount <- question "Value in ethers:"

    -- Notification channel
    notify <- liftIO newChan

    res <- liftIO $ runWeb3 $ do
        owner     <- getAddress "AiraEth.bot"
        builder   <- getAddress "BuilderInvoice.contract"
        comission <- getAddress "ComissionInvoice.contract"
        cost    <- fromWei <$> BInvoice.buildingCostWei builder
        event builder $ \(BInvoice.Builded _ inst) -> do
            res <- runWeb3 (Invoice.beneficiary inst :: Web3 (BytesN 32))
            case res of
                Right ident -> writeChan notify inst >> return TerminateEvent
                _ -> return ContinueEvent
        BInvoice.create builder (cost :: Wei) comission desc ident (toWei (amount :: Ether)) owner

    case res of
        Left e -> return (toMessage (T.pack (show e)))
        Right tx -> do
            yield (toMessage $ "Success transaction " <> etherscan_tx tx
                            <> "\nWaiting for confirmation...")
            inst <- liftIO (readChan notify)
            return (toMessage $
                "Invoice contract created:\n"
                <> etherscan_addr inst)

-- | Create token by Factory
createToken :: AccountedStory

createToken (Account{accountAddress = Nothing}) = return $
    toMessage ("Your account should be verified!" :: Text)

createToken (Account{accountAddress = Just address}) = do
    target <- select "What do you want to create?"
            [ ["ERC20 token"]
            , ["ERC20 token with emission"]
            , ["Ether vault contract"] ]

    name   <- question "Token name (e.g. Ethereum):"
    symbol <- question "Token symbol (e.g. ETH):"

    -- Notification channel
    notify <- liftIO newChan

    case target :: Text of
        "ERC20 token" -> do
            decimal <- question "Count of numbers after point (for integral set 0):"
            total <- question "Amount of tokens on your balance after creation:"

            res <- liftIO $ runWeb3 $ do
                builder <- getAddress "BuilderToken.contract"
                cost    <- fromWei <$> BToken.buildingCostWei builder
                event builder $ \(BToken.Builded client inst) ->
                    case client of
                        address -> writeChan notify inst
                                >> return TerminateEvent
                        _ -> return ContinueEvent
                BToken.create builder (cost :: Wei) name symbol decimal total address

            case res of
                Left e -> return (toMessage (T.pack (show e)))
                Right tx -> do
                    yield (toMessage $ "Success transaction " <> etherscan_tx tx
                                    <> "\nWaiting for confirmation...")
                    inst <- liftIO (readChan notify)
                    return (toMessage $
                        "ERC20 contract created:\n" <> etherscan_addr inst)

        "ERC20 token with emission" -> do
            decimal <- question "Count of numbers after point (for integral set 0):"
            total <- question "Amount of tokens on your balance after creation:"

            res <- liftIO $ runWeb3 $ do
                builder <- getAddress "BuilderTokenEmission.contract"
                cost    <- fromWei <$> BTokenEmission.buildingCostWei builder
                event builder (\(BTokenEmission.Builded client inst) ->
                    case client of
                        address -> writeChan notify inst
                                >> return TerminateEvent
                        _ -> return ContinueEvent)
                BTokenEmission.create builder (cost :: Wei) name symbol decimal total address

            case res of
                Left e -> return (toMessage (T.pack (show e)))
                Right tx -> do
                    yield (toMessage $ "Success transaction " <> etherscan_tx tx
                                    <> "\nWaiting for confirmation...")
                    inst <- liftIO (readChan notify)
                    return (toMessage $
                        "ERC20 with emission contract created:\n"
                        <> etherscan_addr inst)

        "Ether vault contract" -> do
            res <- liftIO $ runWeb3 $ do
                builder <- getAddress "BuilderTokenEther.contract"
                cost    <- fromWei <$> BTokenEther.buildingCostWei builder
                event builder $ \(BTokenEther.Builded client inst) ->
                    case client of
                        address -> writeChan notify inst
                                >> return TerminateEvent
                        _ -> return ContinueEvent
                BTokenEther.create builder (cost :: Wei) name symbol address

            case res of
                Left e -> return (toMessage (T.pack (show e)))
                Right tx -> do
                    yield (toMessage $ "Success transaction " <> etherscan_tx tx
                                    <> "\nWaiting for confirmation...")
                    inst <- liftIO (readChan notify)
                    return (toMessage $
                        "Ether vault contract created:\n"
                        <> etherscan_addr inst)

        _ -> return (toMessage ("Unknown target! Cancelled." :: Text))
