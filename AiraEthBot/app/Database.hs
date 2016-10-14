{-# LANGUAGE FlexibleInstances    #-}
{-# LANGUAGE TemplateHaskell      #-}
{-# LANGUAGE TypeFamilies         #-}
module Database where

import Control.Monad.State (get, put, modify)
import Data.Default.Class (Default(..))
import Control.Monad.Reader (ask)
import Data.IntMap.Strict as I
import Data.Text (Text)
import Data.SafeCopy
import Data.Acid

type EventName    = Text
type ContractName = Text
type ContractAbi  = Text
type Address      = Text
type NotifyID     = Int

data UserDB = UserDB
  { event    :: IntMap (EventName, Address)
  , contract :: IntMap (ContractName, ContractAbi)
  } deriving Show

$(deriveSafeCopy 0 'base ''UserDB)

instance Default UserDB where
    def = UserDB I.empty I.empty

data Database = Database (IntMap UserDB)
  deriving Show

$(deriveSafeCopy 0 'base ''Database)

instance Default Database where
    def = Database I.empty

getUser :: Int -> Query Database UserDB
getUser cid = do Database db <- ask
                 return $ maybe def id $ I.lookup cid db

setUser :: Int -> UserDB -> Update Database ()
setUser cid u = do Database db <- get
                   put $ Database $ I.update (const $ Just u) cid db

$(makeAcidic ''Database ['getUser, 'setUser])
