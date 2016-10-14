module Story where

import Control.Monad.Error.Class (throwError)
import Control.Monad.IO.Class (liftIO)
import Network.Ethereum.Web3.Address
import Data.Text.Read (hexadecimal)
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Web.Telegram.Bot
import Data.Text as T
import ActivationCode
import Data.Acid
import Constants

etherscan_tx :: Text -> Text
etherscan_tx tx = "[" <> tx <> "](https://testnet.etherscan.io/tx/" <> tx <> ")"

toWei :: Double -> Integer
toWei = round . (* 10^18)

fromWei :: Integer -> Double
fromWei = (/ 10^18) . fromIntegral

newtype TelegramAddress = TelegramAddress Address
  deriving Show

instance Answer TelegramAddress where
    parse msg = do
        case text msg of
            Nothing -> throwError "Please send me username."
            Just name -> do
                res <- liftIO $ runWeb3 (getAddress name)
                case res of
                    Right address -> if address == zero
                                     then throwError $ "User `"
                                            <> name <>"` have no address!"
                                     else return (TelegramAddress address)
                    Left e -> throwError (T.pack $ show e)

instance Answer Address where
    parse msg = do
        case text msg of
            Nothing -> throwError "Please send me Ethereum address."
            Just addr -> do
                case fromText addr of
                    Left e -> throwError (T.pack e)
                    Right a -> return a

getAddress :: Text -> Web3 Address
getAddress name = do res <- eth_call (regCall regData) "pending"
                     return $ case fromText (T.drop 26 res) of
                         Right a -> a
                         Left e -> zero
  where regCall = Call Nothing reg_address Nothing Nothing Nothing . Just
        regData = "0x511b1df9" <> paddedInt 32 <> text2data name

getBalance :: Address -> Web3 Integer
getBalance address = do res <- eth_call (airaCall airaData) "pending"
                        return $ case hexadecimal res of
                            Right (x, _) -> x
                            Left e -> 0
  where airaCall = Call Nothing aira_address Nothing Nothing Nothing . Just
        airaData = "0x70a08231" <> paddedAddr (toText address)

withAddress :: Story -> (Address -> Story) -> Story
withAddress storyNoAddress storyAddress c = do
    let Just name = chat_username c
    Right address <- liftIO (runWeb3 $ getAddress name)
    if address == zero
    then storyNoAddress c
    else storyAddress address c

helloAgain :: Text -> Address -> BotMessage
helloAgain name addr = toMessage $ T.unlines
  [ "Hello, " <> name <> ", again!"
  , "You already registered with `" <> toText addr <> "`." ]

hello :: Text -> Text -> BotMessage
hello name code = toMessage $ T.unlines
  [ "Hello, " <> name <> "!"
  , "Your activation code is `" <> code <> "`"
  , "please activate account through Ethereum."
  , "**AiraEtherFunds:** `" <> aira_address <> "`" ]

start :: AcidState ActivationCode -> Story
start db = withAddress genCode $ \address c -> do
    let Just name = chat_first_name c
     in return (helloAgain name address)
  where genCode c = do code <- liftIO (genActivationCode db c)
                       let Just name = chat_first_name c
                        in return (hello name code)

noReg :: Story
noReg _ = do
    return $ toMessage ("You are not registered now!" :: Text)

about :: Story
about = withAddress noReg $ \address c -> do
    let Just name = chat_first_name c
    Right balance <- liftIO (runWeb3 $ getBalance address)
    return $ toMessage $ T.unlines
        [ "Hello, " <> name <> "!"
        , "Your address: `" <> toText address <> "`"
        , "Total balance: " <> pack (show (fromWei balance)) <> " `ether`"
        ]

transfer :: Story
transfer = withAddress noReg $ \address c -> do
    TelegramAddress destination <- question "Recipient user name:"
    amount <- question "Amount of `ether` you want to send:"
    res <- liftIO $ runWeb3 $ do
        let sendCall = Call (Just bot_address) aira_address Nothing Nothing Nothing . Just
            sendData = "0x6d2cb794"
                <> paddedAddr (toText address)
                <> paddedAddr (toText destination)
                <> paddedInt (toWei amount)
        eth_sendTransaction (sendCall sendData)

    return $ toMessage $ case res of
        Left e   -> pack (show e)
        Right tx -> "Transfer success with transaction "
                 <> etherscan_tx tx

send :: Story
send = withAddress noReg $ \address c -> do
    destination <- question "Recipient Ethereum address:"
    amount <- question "Amount of `ether` you want to send:"
    res <- liftIO $ runWeb3 $ do
        let sendCall = Call (Just bot_address) aira_address Nothing Nothing Nothing . Just
            sendData = "0xe1efda6d"
                <> paddedAddr (toText address)
                <> paddedAddr (toText destination)
                <> paddedInt (toWei amount)
        eth_sendTransaction (sendCall sendData)

    return $ toMessage $ case res of
        Left e   -> pack (show e)
        Right tx -> "Send success with transaction "
                 <> etherscan_tx tx
