import { Promise } from 'es6-promise'
import _ from 'lodash'
import { coinbase } from './web3'

export default class Contract {
  constructor(abi, address) {
    this.web3Contract = web3.eth.contract(abi).at(address)
  }

  call(func, args) {
    return new Promise((resolve, reject) => {
      console.log('call', func);
      console.log(args);
      this.web3Contract[func](...args, (error, result) => {
        if (error) {
          reject(error);
        }
        resolve(result);
      })
    });
  }

  send(func, args, txArgs = {}) {
    return new Promise((resolve, reject) => {
      const params = args.concat([
        _.merge({
          from: coinbase()
        }, txArgs)
      ]);
      console.log('send', func);
      console.log(params);
      this.web3Contract[func](...params, (error, result) => {
        if (error) {
          reject(error);
        }
        resolve(result);
      })
    });
  }
}
