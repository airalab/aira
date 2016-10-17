import { combineReducers } from 'redux'
import { reducer as formReducer } from 'redux-form';
import * as modules from '../modules/reducers';

export default combineReducers({
  ...modules,
  form: formReducer
})
