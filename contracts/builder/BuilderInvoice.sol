//
// AIRA Builder for Invoice contract
//

pragma solidity ^0.4.2;
import 'creator/CreatorInvoice.sol';
import 'builder/Builder.sol';

/**
 * @title BuilderInvoice contract
 */
contract BuilderInvoice is Builder {
    /**
     * @dev Run script creation contract
     * @return address new contract
     */
    function create(address _comission, string _description,
                    bytes32 _beneficiary, uint _value,
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
 
        var inst = CreatorInvoice.create(_comission, _description, _beneficiary, _value);
        inst.delegate(_client);
        Builded(_client, inst);
        getContractsOf[_client].push(inst);
        return inst;
    }
}
