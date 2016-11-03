import Promise from 'bluebird'
import _ from 'lodash'
import { coinbase } from './web3'

export default class Contract {
  constructor(abi, address) {
    this.web3Contract = web3.eth.contract(abi).at(address)
  }

  // async callSync(func, args = []) {
  //   const asd = await this.call(func, args);
  //   console.log(asd);
  //   return asd
  // }

  call(func, args = []) {
    return new Promise((resolve, reject) => {
      // console.log('call', func);
      // console.log(args);
      this.web3Contract[func](...args, (error, result) => {
        if (error) {
          reject(error);
        }
        resolve(result);
      })
    });
  }

  get(func, args = []) {
    // console.log('get', func);
    // console.log(args);
    return this.web3Contract[func](...args)
  }

  send(func, args = [], txArgs = {}) {
    return new Promise((resolve, reject) => {
      const paramsGas = args.concat([
        _.merge({
          from: coinbase()
        }, txArgs)
      ]);
      this.web3Contract[func].estimateGas(...paramsGas, (error, gas) => {
        if (error) {
          reject(error);
        }
        console.log(gas);
        const params = args.concat([
          _.merge({
            from: coinbase(),
            gas: gas + 50000
          }, txArgs)
        ]);
        console.log('send', func);
        console.log(params);
        this.web3Contract[func](...params, (errorTx, result) => {
          if (errorTx) {
            reject(errorTx);
          }
          resolve(result);
        })
      })
      // const params = args.concat([
      //   _.merge({
      //     from: coinbase()
      //   }, txArgs)
      // ]);
      // console.log('send', func);
      // console.log(params);
      // this.web3Contract[func](...params, (error, result) => {
      //   if (error) {
      //     reject(error);
      //   }
      //   resolve(result);
      // })
    });
  }

  watch(func) {
    return new Promise((resolve, reject) => {
      const event = this.web3Contract[func]({}, '', (error, result) => {
        if (error) {
          reject(error);
        }
        event.stopWatching()
        resolve(result.args);
      })
    });
  }
}
