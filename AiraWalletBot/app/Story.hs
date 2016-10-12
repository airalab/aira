module Story where

import Control.Monad.IO.Class (liftIO)
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Web.Telegram.Bot
import Data.Text as T
import Data.Acid
import Database

bot_address :: Text
bot_address = "0xb27a2b2b3fe4f2e4bb36d9c03fb8d504c98d8ecb"

reg_address :: Text
reg_address = ""

aira_address :: Text
aira_address = ""

etherscan_tx :: Text -> Text
etherscan_tx = ("https://testnet.etherscan.io/tx/" <>)

toWei :: Double -> Integer
toWei = round . (* 10^18)

about :: AcidState Database -> Story
about _ c = do
    let Just name = chat_username c
    addr <- liftIO $ runWeb3 $ do
        let regCall = Call Nothing reg_address Nothing Nothing Nothing . Just
            regData = "0x511b1df9" <> paddedInt 32 <> text2data name
        eth_call (regCall regData) "latest"

    return $ toMessage $ case addr of
        Left e  -> pack (show e)
        Right a -> "Hello " <> name <> ", your address is " <> T.drop 26 a

send :: AcidState Database -> Story
send db c = do
    dest <- question "Transfer destination:"
    amount <- question "How amount `ether` you want to send?"

    let Just name = chat_username c
    tx <- liftIO $ runWeb3 $ do
        let regCall = Call Nothing reg_address Nothing Nothing Nothing . Just
            regData = "0x511b1df9" <> paddedInt 32 <> text2data name
        Right from <- eth_call (regCall regData) "latest"

        let sendCall = Call (Just bot_address) aira_address Nothing Nothing Nothing . Just
            sendData = "0x23b872dd"
                    <> paddedAddr from
                    <> paddedAddr (T.drop 2 dest)
                    <> paddedInt (toWei amount)
        eth_sendTransaction (sendCall sendData)

    return $ toMessage $ case addr of
        Left e  -> pack (show e)
        Right a -> "Transfer success with transaction ["
                    <> tx <> "](" <> etherscan_tx tx <> ")"

event :: AcidState Database -> Story
event db = undefined

eventOff :: AcidState Database -> Story
eventOff db = undefined
