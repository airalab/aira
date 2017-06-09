pragma solidity ^0.4.4;

import 'market/LiabilityMarket.sol';

/**
 * @title Robit liability marketplace
 */
contract AiraMarket is LiabilityMarket {
    function AiraMarket(string _name, address _taxman, uint256 _comission) LiabilityMarket(_name) {
        taxman    = _taxman;
        comission = _comission;
    }

    /**
     * @dev Taxman account address
     */
    address public taxman;

    /**
     * @dev Taxman comission in percent
     */
    uint256 public comission;

    /**
     * @dev Get tax value by order id
     * @param _id Order identifier
     */
    function taxOf(uint256 _id) constant returns (uint256) {
        return comission * priceOf[_id] / 100;
    }

    /**
     * @dev Make a limit order to buy liability
     * @param _price Liability price
     * @notice Sender is promisor of liability
     */
    function limitBuy(uint256 _price) {
        var id = orders.length++;

        // Store price
        priceOf[id] = _price;
        // Append ask
        putAsk(id);
        // Store template
        orders[id].promisor = msg.sender;

        // Lock tokens
        if (!token.transferFrom(msg.sender, this, _price))
            throw;

        var tax = taxOf(id);
        if (tax == 0 || !token.transferFrom(msg.sender, taxman, tax))
            throw;

        ordersOf[msg.sender].push(id);
        OpenAskOrder(id);
    }

    /**
     * @dev Buy liability
     * @param _id Order index 
     */
    function buyAt(uint256 _id) {
        var o = orders[_id];
        if (_id >= orders.length || o.closed) throw;

        getBid(orderBidOf[_id]);
        o.promisor = msg.sender;

        if (!token.transferFrom(msg.sender, this, priceOf[_id]))
            throw;

        var tax = taxOf(_id);
        if (tax == 0 || !token.transferFrom(msg.sender, taxman, tax))
            throw;

        if (!runLiability(o.beneficiary[0],
                          o.promisee[0],
                          o.promisor,
                          priceOf[_id])) throw;

        o.closed = true;
        CloseBidOrder(_id);
    }
}
