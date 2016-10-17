module Constants where

import Web.Telegram.Bot (BotMessage, toMessage)
import qualified Data.Text as T
import Data.Text (Text)

bot_address :: Text
bot_address = "0xb27a2b2b3fe4f2e4bb36d9c03fb8d504c98d8ecb"

reg_address :: Text
reg_address = "0x1565D3a7a41Ea865f457cC6404A52E43E103eDf5"

aira_address :: Text
aira_address = "0x6EB7e72d7d251ab81b20366949fDbF9F939636C8"

noName :: BotMessage
noName = toMessage $ T.unlines
    [ "Hi, You don't have telegram username."
    , "Please go to telegram app settings and"
    , "add your unique username." ]


noReg :: BotMessage
noReg = toMessage ("You are not registered now!" :: Text)

