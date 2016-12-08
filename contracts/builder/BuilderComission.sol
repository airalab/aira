//
// AIRA Builder for Comission contract
//

pragma solidity ^0.4.2;
import 'creator/CreatorComission.sol';
import 'builder/Builder.sol';

/**
 * @title BuilderComission contract
 */
contract BuilderComission is Builder {
    /**
     * @dev Run script creation contract
     * @return address new contract
     */
    function create(address _ledger, bytes32 _taxman, uint _taxPerc,
                    address _client) payable returns (address) {
        if (buildingCostWei > 0 && beneficiary != 0) {
            // Too low value
            if (msg.value < buildingCostWei) throw;
            // Beneficiary send
            if (!beneficiary.send(buildingCostWei)) throw;
            // Refund
            if (msg.value > buildingCostWei) {
                if (!msg.sender.send(msg.value - buildingCostWei)) throw;
            }
        } else {
            // Refund all
            if (msg.value > 0) {
                if (!msg.sender.send(msg.value)) throw;
            }
        }

        if (_client == 0)
            _client = msg.sender;
 
        var inst = CreatorComission.create(_ledger, _taxman, _taxPerc);
        inst.delegate(_client);
        Builded(_client, inst);
        getContractsOf[_client].push(inst);
        return inst;
    }
}
