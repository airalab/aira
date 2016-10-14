//
// AIRA Builder for OwnedRegistrar contract
//

pragma solidity ^0.4.2;
import 'creator/CreatorOwnedRegistrar.sol';
import 'builder/Builder.sol';

/**
 * @title BuilderOwnedRegistrar contract
 */
contract BuilderOwnedRegistrar is Builder {
    /**
     * @dev Run script creation contract
     * @return address new contract
     */
    function create() returns (address) {
        var inst = CreatorOwnedRegistrar.create();
        Owned(inst).delegate(msg.sender);
        
        deal(inst);
        return inst;
    }
}
