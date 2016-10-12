//
// AIRA Builder for AiraWallet contract
//

pragma solidity ^0.4.2;
import 'creator/CreatorAiraWallet.sol';
import 'builder/Builder.sol';

/**
 * @title BuilderAiraWallet contract
 */
contract BuilderAiraWallet is Builder {
    /**
     * @dev Run script creation contract
     * @return address new contract
     */
    function create(string _name, string _symbol, uint256 _reg_fee, uint256 _limit) returns (address) {
        var inst = CreatorAiraWallet.create(_name, _symbol, _reg_fee, _limit);
        Owned(inst).delegate(msg.sender);
        
        deal(inst);
        return inst;
    }
}
