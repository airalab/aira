-- |
-- Module      :  Aira.Account
-- Copyright   :  Alexander Krupenkin 2016
-- License     :  BSD3
--
-- Maintainer  :  mail@akru.me
-- Stability   :  experimental
-- Portability :  unknown
--
-- AIRA accounting system.
--
module Aira.Account (
    AiraStory
  , Account
  , accounting
  , airaWeb3
  , proxy
  ) where

import Aira.Bot.Proxy
import Aira.TextFormat
import Aira.Registrar
import Aira.Config
import Web.Bot

type Account     = (User, [Proxy])
type AiraStory a = Account -> StoryT (Bot a) Message

-- | User accounting combinator
accounting :: (APIToken a, Persist a) => AiraStory a -> Story a
accounting story user = do
    pxs <- lift $ userProxies user
    case pxs of
        [] -> do px <- createProxy user
                 story (user, [px])
        _  -> story (user, pxs)
