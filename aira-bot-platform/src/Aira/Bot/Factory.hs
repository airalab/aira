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

import Network.Ethereum.Web3.Address (zero)
import Control.Monad.IO.Class (MonadIO(..))
import Control.Monad (liftM2, filterM)
import qualified Data.Text as T
import Control.Concurrent.Chan
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Web.Telegram.Bot
import Data.Text (Text)
import Control.Arrow
import Pipes (yield)

import qualified Aira.Contract.Invoice              as Invoice
import qualified Aira.Contract.BuilderInvoice       as BInvoice
import qualified Aira.Contract.BuilderToken         as BToken
import qualified Aira.Contract.BuilderTokenEther    as BTokenEther
import qualified Aira.Contract.BuilderTokenEmission as BTokenEmission
import Aira.Bot.Common
import Aira.Registrar
import Aira.Account

tryWhileM :: MonadIO m => [Web3 DefaultProvider b] -> m [b]
tryWhileM = go []
  where go acc [] = return (reverse acc)
        go acc (x : xs) = do
            res <- runWeb3' x
            case res of
                Right a -> go (a : acc) xs
                Left _ -> return (reverse acc)

invoiceGuard :: AccountedStory -> AccountedStory
invoiceGuard story a@(Account{accountHash = client}) = do
    Right bot     <- runWeb3 $ getAddress "AiraEth.bot"
    Right builder <- runWeb3 $ getAddress "BuilderInvoice.contract"
    invoices      <- tryWhileM (BInvoice.getContractsOf builder bot <$> [0..])
    Right opened  <- runWeb3 $ filterM (openBy client) invoices
    if length opened < 2
    then story a
    else return $ toMessage $ T.unlines
            [ "You already have TWO opened invoices!"
            , "Is no more allowed, sorry." ]
  where
    openBy x = uncurry (liftM2 (&&)) . (isOpen &&& isBeneficiary x)
    isBeneficiary x = fmap (x ==) . Invoice.beneficiary
    isOpen = fmap (== zero) . Invoice.signer

-- | Create Invoice contract by Factory
createInvoice :: AccountedStory
createInvoice = invoiceGuard $ \Account{accountHash = ident} -> do
    desc <- question "Description:"
    amount <- question "Value in ethers:"

    -- Notification channel
    notify <- liftIO newChan

    res <- runWeb3 $ do
        owner     <- getAddress "AiraEth.bot"
        builder   <- getAddress "BuilderInvoice.contract"
        comission <- getAddress "ComissionInvoice.contract"
        cost    <- fromWei <$> BInvoice.buildingCostWei builder
        event builder $ \(BInvoice.Builded _ inst) -> do
            res <- runWeb3 (Invoice.beneficiary inst)
            case res of
                Right a ->
                    if a == ident
                    then writeChan notify inst >> return TerminateEvent
                    else return ContinueEvent
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

            res <- runWeb3 $ do
                builder <- getAddress "BuilderToken.contract"
                cost    <- fromWei <$> BToken.buildingCostWei builder
                event builder $ \(BToken.Builded client inst) ->
                    if client == address
                    then writeChan notify inst >> return TerminateEvent
                    else return ContinueEvent
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

            res <- runWeb3 $ do
                builder <- getAddress "BuilderTokenEmission.contract"
                cost    <- fromWei <$> BTokenEmission.buildingCostWei builder
                event builder (\(BTokenEmission.Builded client inst) ->
                    if client == address
                    then writeChan notify inst >> return TerminateEvent
                    else return ContinueEvent)
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
            res <- runWeb3 $ do
                builder <- getAddress "BuilderTokenEther.contract"
                cost    <- fromWei <$> BTokenEther.buildingCostWei builder
                event builder $ \(BTokenEther.Builded client inst) ->
                    if client == address
                    then writeChan notify inst >> return TerminateEvent
                    else return ContinueEvent
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
