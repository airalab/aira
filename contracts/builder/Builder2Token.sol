//
// AIRA Builder v2 for Token contract
//
// Ethereum address:
//  - Mainnet:
//  - Testnet: 
//

pragma solidity ^0.4.2;
import 'creator/CreatorToken.sol';
import './Builder2.sol';

/**
 * @title Builder2Token contract
 */
contract Builder2Token is Builder2 {
    /**
     * @dev Run script creation contract
     * @param _client is a builder client
     * @param _name is name token
     * @param _symbol is symbol token
     * @param _decimals is fixed point position
     * @param _count is count of tokens exist
     * @return address new contract
     */
    function create(address _client, string _name, string _symbol, uint8 _decimals, uint256 _count) returns (address) {
        if (buildingCostWei > 0 && beneficiary != 0 && address(credit) != 0) {
            if (!credit.transferFrom(_client, beneficiary, buildingCostWei))
                throw;
        }
 
        var inst = CreatorToken.create(_name, _symbol, _decimals, _count);
        getContractsOf[_client].push(inst);
        Builded(_client, inst);
        inst.transfer(_client, _count);
        inst.delegate(_client);
        return inst;
    }
}
