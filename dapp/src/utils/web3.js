import Promise from 'bluebird'
import _ from 'lodash'
import axios from 'axios'
import Blockchain from './blockchain'
import abis from './abi'
import Contract from './contract'

export function getWeb3() {
  if (typeof web3 !== 'undefined' && typeof Web3 !== 'undefined') {
    web3 = new Web3(web3.currentProvider);
  } else if (typeof Web3 !== 'undefined') {
    web3 = new Web3(new Web3.providers.HttpProvider('http://localhost:8545'));
  } else if (typeof web3 === 'undefined' && typeof Web3 === 'undefined') {
    return false
  }
  return web3
}

export function getAccounts() {
  return web3.eth.accounts
}

let indexAccount = 0
export function setAccount(index) {
  indexAccount = index
}

export function coinbase() {
  return web3.eth.accounts[indexAccount]
}

export function getBalance(address) {
  return parseFloat(web3.fromWei(web3.eth.getBalance(address), 'ether').toString())
}

export function isAccounts() {
  if (web3.eth.accounts.length > 0) {
    return true
  }
  return false
}

export function transfer(from, to, value, isEther = true) {
  return new Promise((resolve, reject) => {
    web3.eth.sendTransaction({
      from,
      to,
      value: (isEther) ? web3.toWei(value, 'ether') : value
    }, (error, result) => {
      if (error) {
        reject(error);
      }
      resolve(result);
    })
  });
}

export function getTransaction(txId) {
  return web3.eth.getTransaction(txId);
}

export function getContract(abi, address) {
  return new Contract(abi, address)
}

export function getUrlAbi(contract) {
  const contractNew = contract.split(' ').pop()
  if (/builder/i.test(contractNew)) {
    return 'https://raw.githubusercontent.com/airalab/core/master/abi/builder/' + contractNew + '.json'
  }
  return 'https://raw.githubusercontent.com/airalab/core/master/abi/modules/' + contractNew + '.json'
}

function loadAbi(url) {
  return axios.get(url)
    .then(results => results.data);
}

export function loadAbiByName(name) {
  if (_.has(abis, name)) {
    return new Promise((resolve) => {
      resolve(abis[name]);
    });
  }
  return loadAbi(getUrlAbi(name));
}

export const blockchain = new Blockchain(getWeb3())
