import { FLASH_MESSAGE, SET_BALANCE, SET_APPROVED } from './actionTypes'

const initialState = {
  flash_message: '',
  balance: 0,
  approved: 0
}

export default function app(state = initialState, action) {
  switch (action.type) {
    case FLASH_MESSAGE:
      return { ...state, flash_message: action.payload }

    case SET_BALANCE:
      return { ...state, balance: action.payload }

    case SET_APPROVED:
      return { ...state, approved: action.payload }

    default:
      return state;
  }
}
