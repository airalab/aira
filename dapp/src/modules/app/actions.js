import { startSubmit, stopSubmit, reset } from 'redux-form';
import { FLASH_MESSAGE, SET_BALANCE, SET_APPROVED } from './actionTypes'
import { ADDRESS, ADDRESS_BOT } from '../../config/config'
import { loadAbiByName, getContract, blockchain, tx, getWeb3, transfer, coinbase } from '../../utils/web3'

export function flashMessage(message) {
  return {
    type: FLASH_MESSAGE,
    payload: message
  }
}

export function load() {
  return (dispatch) => {
    loadAbiByName('AiraEtherFunds')
      .then((abi) => {
        const contract = getContract(abi, ADDRESS);
        dispatch({
          type: SET_BALANCE,
          payload: getWeb3().fromWei(contract.balanceOf(coinbase()), 'ether')
        })
        dispatch({
          type: SET_APPROVED,
          payload: getWeb3().fromWei(contract.allowance(coinbase(), ADDRESS_BOT), 'ether')
        })
      })
  }
}

function run(dispatch, address, abiName, action, values, txArgs = {}) {
  return loadAbiByName(abiName)
    .then((abi) => {
      const contract = getContract(abi, address);
      return tx(contract, action, values, txArgs)
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
    run(dispatch, ADDRESS, 'AiraEtherFunds', 'approve', [ADDRESS_BOT, getWeb3().toWei(form.value, 'ether')])
      .then((blockNumber) => {
        dispatch(stopSubmit('FormApprove'))
        dispatch(reset('FormApprove'))
        dispatch(flashMessage('blockNumber: ' + blockNumber))
      })
      .catch((e) => {
        console.log(e);
        dispatch(stopSubmit('FormApprove'))
      })
  }
}

export function submitSend(form) {
  return (dispatch) => {
    dispatch(startSubmit('FormSend'));
    const txId = transfer(coinbase(), ADDRESS, form.value);
    dispatch(flashMessage('txId: ' + txId));
    blockchain.subscribeTx(txId)
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
