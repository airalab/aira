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
     * @dev _bot_reg is a registrar of bots
     * @dev _name is token name
     * @dev _symbol is token symbol
     * @return address new contract
     */
    function create(address _bot_reg, string _name, string _symbol) payable returns (address) {
        var inst = CreatorAiraEtherFunds.create(_bot_reg, _name, _symbol);
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
