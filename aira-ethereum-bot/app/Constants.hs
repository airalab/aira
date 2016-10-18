module Constants where

import Web.Telegram.Bot (BotMessage, toMessage)
import qualified Data.Text as T
import Data.Text (Text)

bot_address :: Text
bot_address = "0xce7acbd1f75c03aaa4f6ad4099f2edd51a1b4309"

reg_address :: Text
reg_address = ""

aira_address :: Text
aira_address = ""

noName :: BotMessage
noName = toMessage $ T.unlines
    [ "Hi, You don't have telegram username."
    , "Please go to telegram app settings and"
    , "add your unique username." ]


noReg :: BotMessage
noReg = toMessage ("You are not registered now!" :: Text)
