{-# LANGUAGE FlexibleInstances  #-}
{-# LANGUAGE TemplateHaskell    #-}
{-# LANGUAGE TypeFamilies       #-}
-- |
-- Module      :  Aira.Bot.Activation
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  portable
--
-- Aira code based account activation service.
--
module Aira.Bot.Activation (
    ActivationCode
  , accountAddress
  , listenCode
  , genCode
  ) where

import Web.Telegram.Bot (forkBot, sendMessageBot, toMessage)
import Web.Telegram.API.Bot (Chat(..), ChatType(..))
import Data.Time (UTCTime, getCurrentTime)
import Network.Ethereum.Web3.Address as A
import Control.Concurrent (threadDelay)
import Data.Default.Class (Default(..))
import Control.Monad.IO.Class (liftIO)
import Web.Telegram.Bot.Types (Bot)
import Control.Monad.Reader (ask)
import qualified Data.Text as T
import qualified Data.Map as M
import Data.Text (Text, pack)
import Network.Ethereum.Web3
import Data.Monoid ((<>))
import Aira.Registrar
import Data.Map (Map)
import System.Random
import Data.SafeCopy
import Data.Acid

import Lens.Family2.State
import Lens.Family2.TH
import Lens.Family2

codeLength :: Num a => a
codeLength = 6

codeRange :: String
codeRange = ['0'..'9'] ++ ['a'..'z'] ++ ['A'..'Z']

type Code = Text

$(deriveSafeCopy 0 'base ''Chat)
$(deriveSafeCopy 0 'base ''ChatType)

instance Ord Chat where
    compare a b = compare (chat_id a) (chat_id b)

instance Eq Chat where
    a == b = chat_id a == chat_id b

data ActivationCode = ActivationCode
  { _chats :: Map Code Chat
  , _codes :: Map Chat Code
  , _times :: Map Code UTCTime
  } deriving Show

instance Default ActivationCode where
    def = ActivationCode M.empty M.empty M.empty

$(makeLenses ''ActivationCode)
$(deriveSafeCopy 0 'base ''ActivationCode)

newCode :: Code -> Chat -> UTCTime -> Update ActivationCode ()
newCode code chat time = do
    chats %= M.insert code chat
    codes %= M.insert chat code
    times %= M.insert code time

deleteCode :: Code -> Update ActivationCode ()
deleteCode code = do
    Just chat <- chats `uses` M.lookup code
    chats %= M.delete code
    codes %= M.delete chat
    times %= M.delete code

getCode :: Chat -> Query ActivationCode (Maybe Code)
getCode chat = codes `views` M.lookup chat <$> ask

getChat :: Code -> Query ActivationCode (Maybe Chat)
getChat code = chats `views` M.lookup code <$> ask

getTime :: Code -> Query ActivationCode (Maybe UTCTime)
getTime code = times `views` M.lookup code <$> ask

$(makeAcidic ''ActivationCode ['newCode, 'deleteCode, 'getCode, 'getChat, 'getTime])

activationFilter :: Address -> Filter
activationFilter emitter =
    Filter (Just ("0x" <> A.toText emitter))
           (Just [ Just "0x87283f0fd3af976c1c41e7d549d4b95f8f812b475d4b68fa8e1db59db0391c94"
                 , Nothing])
           Nothing
           Nothing

handleActivation :: AcidState ActivationCode -> Change -> Bot ()
handleActivation db (Change {changeTopics = topics}) = do
    let Right address = fromText (T.drop 26 $ topics !! 1)
        code = unhex (T.take (codeLength * 2) $ T.drop 2 $ topics !! 2)
    mbChat <- liftIO $ query db (GetChat code)
    case mbChat of
        Nothing -> return ()
        Just chat -> do
            let Just first_name = chat_first_name chat
                Just name = chat_username chat
            liftIO $ do
                update db (DeleteCode code)
                runWeb3 $ setAddress ("account-" <> T.toLower name) address
            sendMessageBot chat $
                toMessage $ T.unlines [ "A good news, " <> first_name <> "!"
                                      , "Activation code received, unlocking..."
                                      , "Wait a bit to give you a power. /me" ]

-- | Listening events from AiraEtherFunds and try to activate account
listenCode :: AcidState ActivationCode -> Bot ()
listenCode db = do
    forkBot $ do
        Right actFilterId <- liftIO $ runWeb3 $
            resolve "contract-AiraEtherFunds" >>= eth_newFilter . activationFilter
        let loop = do
             Right upd <- liftIO $ runWeb3 (eth_getFilterChanges actFilterId)
             mapM_ (handleActivation db) upd
             liftIO (threadDelay 10000000)
             loop
         in loop
    return ()

-- | Make activation code for account or return already exist
genCode :: AcidState ActivationCode -> Chat -> IO Text
genCode db chat = do
    mbCode <- query db (GetCode chat)
    case mbCode of
        Just code -> return code
        Nothing   -> do
            g <- newStdGen
            let seed = (codeRange !!) <$> randomRs (0, length codeRange - 1) g
                code = pack (take codeLength seed)
            time <- getCurrentTime
            update db (NewCode code chat time)
            return code

-- | Take address by account name
accountAddress :: Text -> Web3 Address
accountAddress name = resolve ("account-" <> T.toLower name)
