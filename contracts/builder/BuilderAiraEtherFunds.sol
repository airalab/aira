//
// AIRA Builder for AiraEtherFunds contract
//

pragma solidity ^0.4.2;
import 'creator/CreatorAiraEtherFunds.sol';
import 'builder/Builder.sol';

/**
 * @title BuilderAiraEtherFunds contract
 */
contract BuilderAiraEtherFunds is Builder {
    /**
     * @dev Run script creation contract
     * @dev _name is token name
     * @dev _symbol is token symbol
     * @return address new contract
     */
    function create(string _name, string _symbol) returns (address) {
        var inst = CreatorAiraEtherFunds.create(_name, _symbol);
        Owned(inst).delegate(msg.sender);
        
        deal(inst);
        return inst;
    }
}
