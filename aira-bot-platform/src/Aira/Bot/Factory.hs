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
    createToken
  ) where

import qualified Data.Text                          as T
import qualified Aira.Contract.BuilderToken         as BToken
import qualified Aira.Contract.BuilderTokenEther    as BTokenEther
import qualified Aira.Contract.BuilderTokenEmission as BTokenEmission
import Control.Concurrent.Chan
import Control.Monad.IO.Class
import Network.Ethereum.Web3
import Web.Telegram.Bot
import Aira.Bot.Common
import Aira.TextFormat
import Aira.Registrar
import Aira.Account
import Pipes (yield)

-- | Create token by Factory
createToken :: AiraStory
createToken (_, _, px : _) = do
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

            res <- airaWeb3 $ do
                builder <- getAddress "BuilderToken.contract"
                cost    <- fromWei <$> BToken.buildingCostWei builder

                tx <- proxy px builder (cost :: Wei) $
                    BToken.CreateData name symbol decimal total px

                event builder $ \(BToken.Builded client inst) ->
                    if client == px
                    then writeChan notify inst >> return TerminateEvent
                    else return ContinueEvent

                return tx

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

            res <- airaWeb3 $ do
                builder <- getAddress "BuilderTokenEmission.contract"
                cost    <- fromWei <$> BTokenEmission.buildingCostWei builder

                tx <- proxy px builder (cost :: Wei) $
                    BTokenEmission.CreateData name symbol decimal total px

                event builder (\(BTokenEmission.Builded client inst) ->
                    if client == px
                    then writeChan notify inst >> return TerminateEvent
                    else return ContinueEvent)

                return tx

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
            res <- airaWeb3 $ do
                builder <- getAddress "BuilderTokenEther.contract"
                cost    <- fromWei <$> BTokenEther.buildingCostWei builder

                tx <- proxy px builder (cost :: Wei) $
                    BTokenEther.CreateData name symbol px

                event builder $ \(BTokenEther.Builded client inst) ->
                    if client == px
                    then writeChan notify inst >> return TerminateEvent
                    else return ContinueEvent

                return tx


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
