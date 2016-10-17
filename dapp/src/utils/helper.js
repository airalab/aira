import _ from 'lodash'
import { getContract as web3Contract, loadAbiByName, coinbase } from './web3';

export function getContract(name, address) {
  return loadAbiByName(name)
    .then(abi => web3Contract(abi, address))
}

export function getMarketLots(address) {
  let market
  let tokenAbi
  return getContract('Market', address)
    .then((contract) => {
      market = contract
      return loadAbiByName('TokenEmission')
    })
    .then((abi) => {
      tokenAbi = abi
      return loadAbiByName('Lot')
    })
    .then((abi) => {
      const lots = []
      for (let addr = market.first(); addr !== '0x0000000000000000000000000000000000000000' && addr !== 0; addr = market.next(addr)) {
        const lot = web3Contract(abi, addr)

        if (!lot.closed()) {
          const sale = lot.sale()
          const buy = lot.buy()
          const tokenSale = web3Contract(tokenAbi, sale)
          const tokenBuy = web3Contract(tokenAbi, buy)

          let saleApprove = _.toNumber(tokenSale.allowance(lot.seller(), addr))
          const saleBalance = _.toNumber(tokenSale.balanceOf(lot.seller()))
          saleApprove = saleApprove > saleBalance ? saleBalance : saleApprove;

          let buyApprove = _.toNumber(tokenBuy.allowance(coinbase(), addr))
          const buyBalance = _.toNumber(tokenBuy.balanceOf(coinbase()))
          buyApprove = buyApprove > buyBalance ? buyBalance : buyApprove;

          try {
            lots.push({
              address: addr,
              seller: lot.seller(),
              buyer: lot.buyer(),
              sale_address: sale,
              buy_address: buy,
              sale_name: tokenSale.name(),
              buy_name: tokenBuy.name(),
              sale_quantity: _.toNumber(lot.quantity_sale()),
              buy_quantity: _.toNumber(lot.quantity_buy()),
              approve_sale_quantity: saleApprove,
              approve_buy_quantity: buyApprove,
              my: lot.seller() === coinbase()
            })
          } catch (e) {
            console.log('load lot err token info', e);
          }
        }
      }
      return lots
    })
}
