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
    function create() payable returns (address) {
        var inst = CreatorOwnedRegistrar.create();
        Owned(inst).delegate(msg.sender);
        getContractsOf[msg.sender].push(inst);

        if (buildingCostWei > 0 && beneficiary != 0) {
            if (   msg.value < buildingCostWei
               || !msg.sender.send(msg.value - buildingCostWei)
               || !beneficiary.send(buildingCostWei)
               ) throw;
        }
        
        Builded(msg.sender, inst);
        return inst;
    }
}
