import { startSubmit, stopSubmit, reset } from 'redux-form';
import _ from 'lodash';
import { FLASH_MESSAGE, SET_BALANCE, SET_APPROVED } from './actionTypes'
import { ADDRESS } from '../../config/config'
import { loadAbiByName, getContract, blockchain, getWeb3, transfer, coinbase } from '../../utils/web3'

export function flashMessage(message) {
  return {
    type: FLASH_MESSAGE,
    payload: message
  }
}

export function load() {
  return {
    type: 'LOAD'
  }
}

export function getApprovedByAddress(token, target) {
  return (dispatch) => {
    loadAbiByName('TokenEmission')
      .then((abi) => {
        let decimals
        const contract = getContract(abi, token);
        contract.call('decimals')
          .then((result) => {
            decimals = result
            if (decimals > 0) {
              decimals = Math.pow(10, decimals)
            } else {
              decimals = 1
            }
            return contract.call('allowance', [coinbase(), target])
          })
          .then((result) => {
            dispatch({
              type: SET_APPROVED,
              payload: _.toNumber(result) / decimals
            })
          })
      })
  }
}

export function getBalanceByToken(token) {
  return (dispatch) => {
    loadAbiByName('TokenEmission')
      .then((abi) => {
        let decimals
        const contract = getContract(abi, token);
        contract.call('decimals')
          .then((result) => {
            decimals = result
            if (decimals > 0) {
              decimals = Math.pow(10, decimals)
            } else {
              decimals = 1
            }
            return contract.call('balanceOf', [coinbase()])
          })
          .then((result) => {
            dispatch({
              type: SET_BALANCE,
              payload: _.toNumber(result) / decimals
            })
          })
      })
  }
}

function run(dispatch, address, abiName, action, values, txArgs = {}) {
  return loadAbiByName(abiName)
    .then((abi) => {
      const contract = getContract(abi, address);
      return contract.send(action, values, txArgs)
    })
    .then((txId) => {
      dispatch(flashMessage('txId: ' + txId))
      return blockchain.subscribeTx(txId)
    })
    .then(transaction => transaction.blockNumber)
}

export function submitIdentify(form) {
  return (dispatch) => {
    dispatch(startSubmit('FormIdentify'));
    run(dispatch, ADDRESS, 'AiraEtherFunds', 'activate', [form.code], { value: getWeb3().toWei(form.value, 'ether') })
      .then((blockNumber) => {
        dispatch(stopSubmit('FormIdentify'))
        dispatch(reset('FormIdentify'))
        dispatch(flashMessage('blockNumber: ' + blockNumber))
      })
      .catch((e) => {
        console.log(e);
        dispatch(stopSubmit('FormIdentify'))
      })
  }
}

export function submitApprove(form) {
  return (dispatch) => {
    dispatch(startSubmit('FormApprove'));
    loadAbiByName('TokenEmission')
      .then((abi) => {
        let decimals
        const contract = getContract(abi, form.address_token);
        contract.call('decimals')
          .then((result) => {
            decimals = result
            if (decimals > 0) {
              decimals = Math.pow(10, decimals)
            } else {
              decimals = 1
            }
            const value = _.toNumber(form.value) / decimals
            return run(dispatch, form.address_token, 'TokenEmission', 'approve', [form.address_target, value])
          })
          .then((blockNumber) => {
            dispatch(stopSubmit('FormApprove'))
            dispatch(getApprovedByAddress(form.address_token, form.address_target))
            dispatch(flashMessage('blockNumber: ' + blockNumber))
          })
          .catch((e) => {
            console.log(e);
            dispatch(stopSubmit('FormApprove'))
          })
      })
  }
}

export function submitSend(form) {
  return (dispatch) => {
    dispatch(startSubmit('FormSend'));
    transfer(coinbase(), ADDRESS, form.value)
      .then((txId) => {
        dispatch(flashMessage('txId: ' + txId));
        return blockchain.subscribeTx(txId)
      })
      .then(transaction => transaction.blockNumber)
      .then((blockNumber) => {
        dispatch(stopSubmit('FormSend'))
        dispatch(reset('FormSend'))
        dispatch(flashMessage('blockNumber: ' + blockNumber))
      })
      .catch((e) => {
        console.log(e);
        dispatch(stopSubmit('FormSend'))
      })
  }
}
