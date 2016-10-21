module Constants where

import Web.Telegram.Bot (BotMessage, toMessage)
import qualified Data.Text as T
import Data.Text (Text)

bot_address :: Text
bot_address = "0xce7acbd1f75c03aaa4f6ad4099f2edd51a1b4309"

reg_address :: Text
reg_address = "0xE5322B2b1A512Ba8DbAe458E7f0eF38C743C93b9"

aira_address :: Text
aira_address = "0xdcf691860758F2f60F40De93AfA6E0814c3826c5"

noName :: BotMessage
noName = toMessage $ T.unlines
    [ "Hi, You don't have telegram username."
    , "Please go to telegram app settings and"
    , "add your unique username." ]


noReg :: BotMessage
noReg = toMessage ("You are not registered now!" :: Text)
