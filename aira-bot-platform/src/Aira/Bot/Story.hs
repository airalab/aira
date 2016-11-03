{-# LANGUAGE FlexibleContexts #-}
-- |
-- Module      :  Aira.Bot.Story
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- Aira common bot stories.
--
module Aira.Bot.Story (
    AccountAddress(..)
  , etherscan_addr
  , etherscan_tx
  , floatToText
  , withUsername
  , withAddress
  , withFee
  , unregister
  , noRegStory
  , noName
  , start
  , about
  ) where

import Control.Monad.Error.Class (throwError)
import Data.Text.Lazy.Builder (toLazyText)
import Data.Text.Lazy.Builder.RealFloat
import Control.Monad.IO.Class (liftIO)
import Network.Ethereum.Web3.Address
import Data.Text.Read (hexadecimal)
import Control.Applicative ((<|>))
import Data.Text.Lazy (toStrict)
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Web.Telegram.Bot
import Data.Text as T
import Data.Acid

import Aira.Bot.Activation
import Aira.Bot.Contract
import Aira.Registrar

fee = 0.01

withFee :: Address -> Double -> Web3 Text -> Web3 Text
withFee address amount fun = do
    beneficiary <- resolve "AiraEth.bot"
    balance     <- getBalance address
    if amount + fee > balance
    then
        throwError $ UserFail "Allowed balance is too low for this operation!"
    else
        sendFrom address beneficiary fee >> fun

etherscan_tx :: Text -> Text
etherscan_tx tx = "[" <> tx <> "](https://etherscan.io/tx/" <> tx <> ")"

etherscan_addr :: Text -> Text
etherscan_addr a = "[" <> a <> "](https://etherscan.io/address/" <> a <> ")"

floatToText :: RealFloat a => a -> Text
floatToText = toStrict . toLazyText . formatRealFloat Fixed Nothing

newtype AccountAddress = AccountAddress Address
  deriving Show

instance Answer AccountAddress where
    parse msg = case text msg of
        Nothing -> throwError "Please send me username."
        Just name -> go name =<< liftIO (runWeb3 $ accountAddress name)
      where go name (Right address)
                | address == zero = throwError $ "User `" <> name <> "` have no address!"
                | otherwise       = return (AccountAddress address)
            go _ (Left e) = throwError (T.pack $ show e)

instance Answer Address where
    parse msg = case text msg of
        Nothing -> throwError "Please send me Ethereum address."
        Just addr -> case fromText addr of
            Left e -> throwError (T.pack e)
            Right a -> return a

withUsername :: BotMessage -> (Text -> Story) -> Story
withUsername msg story c =
    case chat_username c of
        Just name -> story name c
        Nothing -> return msg

withAddress :: Story -> (Address -> Story) -> Text -> Story
withAddress storyNoAddress storyAddress name c = do
    Right address <- liftIO (runWeb3 $ accountAddress name)
    if address == zero
    then storyNoAddress c
    else storyAddress address c

getName :: Chat -> Text
getName c = name
  where Just name = chat_first_name c <|> chat_username c

noName :: BotMessage
noName = toMessage $ T.unlines
    [ "Hi, You don't have telegram username."
    , "Please go to telegram app settings and"
    , "add your unique username." ]

noReg :: BotMessage
noReg = toMessage ("You are not registered now!" :: Text)

helloAgain :: Address -> Chat -> BotMessage
helloAgain addr c = toMessage $ T.unlines
  [ "Hello, " <> getName c <> ", again!"
  , "I known you as " <> etherscan_addr (toText addr) <> "." ]

hello :: Chat -> Text -> BotMessage
hello c code = toMessage $ T.unlines
  [ "Hello, " <> getName c <> "!"
  , "Your Ethereum address activation code is `" <> code <> "`"
  , "call `activate` of **AiraEtherFunds**" ]

start :: AcidState ActivationCode -> Story
start db = withUsername noName
         $ withAddress noAddress (\x -> return . helloAgain x)
  where noAddress c = do code <- liftIO (genCode db c)
                         return (hello c code)

noRegStory :: Story
noRegStory _ = return noReg

about :: Story
about = withUsername noName
      $ withAddress noRegStory
      $ \address c -> do
          let balance = (,,) <$> getBalance address
                             <*> balanceOf address
                             <*> ethBalance address
          Right (x, y, z) <- liftIO (runWeb3 balance)
          return $ toMessage $ T.unlines
            [ "Hello, " <> getName c <> "!"
            , "Your address: " <> etherscan_addr (toText address)
            , "Balance: " <> floatToText x <> " `ETH` approved / "
                          <> floatToText y <> " `ETH` on contract / "
                          <> floatToText z <> " `ETH` on account"
            ]

unregister :: Story
unregister = withUsername noName
           $ withAddress noRegStory
           $ \_ c -> do
               res <- question $ T.unlines [ "Do you want to delete account address?"
                                           , "Send me 'Do as I say!' to confirm." ]
               case res :: Text of
                  "Do as I say!" -> do
                     let Just name = chat_username c
                     r <- liftIO $ runWeb3 (accountDelete name)
                     return . toMessage $ case r of
                         Right _ -> "Account deleted"
                         Left e -> pack $ show e
                  _ -> return (toMessage ("No confirmation given" :: Text))

secure :: Story
secure _ = return . toMessage $ T.unlines
-- TODO: Fill text description
    [ "@AiraSecureBot" ]
