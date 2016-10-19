module Story where

import Control.Monad.Error.Class (throwError)
import Control.Monad.IO.Class (liftIO)
import Network.Ethereum.Web3.Address
import Data.Text.Read (hexadecimal)
import Control.Applicative ((<|>))
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Web.Telegram.Bot
import Data.Text as T
import ActivationCode
import Data.Acid
import Constants

etherscan_tx :: Text -> Text
etherscan_tx tx = "[" <> tx <> "](https://testnet.etherscan.io/tx/" <> tx <> ")"

etherscan_addr :: Text -> Text
etherscan_addr a = "[" <> a <> "](https://testnet.etherscan.io/address/" <> a <> ")"

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
        regData = "0x511b1df9" <> paddedInt 32 <> text2data (T.toLower name)

getBalance :: Address -> Web3 Integer
getBalance address = do res <- eth_call (airaCall airaData) "pending"
                        return $ case hexadecimal res of
                            Right (x, _) -> x
                            Left e -> 0
  where airaCall = Call Nothing aira_address Nothing Nothing Nothing . Just
        airaData = "0x70a08231" <> paddedAddr (toText address)

withUsername :: BotMessage -> (Text -> Story) -> Story
withUsername msg story c =
    case chat_username c of
        Just name -> story name c
        Nothing -> return msg

withAddress :: Story -> (Address -> Story) -> Text -> Story
withAddress storyNoAddress storyAddress name c = do
    Right address <- liftIO (runWeb3 $ getAddress name)
    if address == zero
    then storyNoAddress c
    else storyAddress address c

getName :: Chat -> Text
getName c = name
  where Just name = chat_first_name c <|> chat_username c

helloAgain :: Chat -> Address -> BotMessage
helloAgain c addr = toMessage $ T.unlines
  [ "Hello, " <> getName c <> ", again!"
  , "I known you as " <> etherscan_addr (toText addr) <> "." ]

hello :: Chat -> Text -> BotMessage
hello c code = toMessage $ T.unlines
  [ "Hello, " <> getName c <> "!"
  , "Your Ethereum address activation code is `" <> code <> "`"
  , "call `activate` of **AiraEtherFunds** at " <> etherscan_addr aira_address ]

start :: AcidState ActivationCode -> Story
start db = withUsername noName $ withAddress genCode $
    \address c -> return (helloAgain c address)
  where genCode c = do code <- liftIO (genActivationCode db c)
                       return (hello c code)

noRegStory :: Story
noRegStory _ = return noReg

about :: Story
about = withUsername noName $ withAddress noRegStory $ \address c -> do
    Right balance <- liftIO (runWeb3 $ getBalance address)
    return $ toMessage $ T.unlines
        [ "Hello, " <> getName c <> "!"
        , "Your address: " <> etherscan_addr (toText address)
        , "Total balance: " <> pack (show (fromWei balance)) <> " `ETH`"
        ]

transfer :: Story
transfer = withUsername noName $ withAddress noRegStory $ \address c -> do
    TelegramAddress destination <- question "Recipient username:"
    amount <- question "Amount of `ether` you want to send:"
    if amount > 0 then do
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
    else return $ toMessage ("I can't transfer zero tokens!" :: Text)

send :: Story
send = withUsername noName $ withAddress noRegStory $ \address c -> do
    destination <- question "Recipient Ethereum address:"
    amount <- question "Amount of `ether` you want to send:"
    if amount > 0 then do
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
    else return $ toMessage ("I can't send zero tokens!" :: Text)

unregister :: Story
unregister = withUsername noName $ \name c -> do
    res <- question $ T.unlines [ "Do you want to delete account address?"
                                , "Send me 'Do as I say!' to confirm." ]
    case res :: Text of
        "Do as I say!" -> do
            r <- liftIO (runWeb3 $ setAccount (T.toLower name) zero)
            return . toMessage $ case r of
                Right _ -> "Account deleted"
                Left e -> pack $ show e
        _ -> return (toMessage ("No confirmation given" :: Text))
