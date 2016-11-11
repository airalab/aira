//
// AIRA Builder v2 for TokenEmission contract
//
// Ethereum address:
//  - Mainnet:
//  - Testnet: 
//

pragma solidity ^0.4.2;
import 'creator/CreatorTokenEmission.sol';
import './Builder2.sol';

/**
 * @title Builder2TokenEmission contract
 */
contract Builder2TokenEmission is Builder2 {
    /**
     * @dev Run script creation contract
     * @param _client is a builder client
     * @param _name is name token
     * @param _symbol is symbol token
     * @param _decimals is fixed point position
     * @param _start_count is count of tokens exist
     * @return address new contract
     */
    function create(address _client, string _name, string _symbol, uint8 _decimals, uint256 _start_count) returns (address) {
        if (buildingCostWei > 0 && beneficiary != 0 && address(credit) != 0) {
            if (!credit.transferFrom(_client, beneficiary, buildingCostWei))
                throw;
        }
 
        var inst = CreatorTokenEmission.create(_name, _symbol, _decimals, _start_count);
        getContractsOf[_client].push(inst);
        Builded(_client, inst);
        inst.transfer(_client, _start_count);
        inst.delegate(_client);
        return inst;
    }
}
