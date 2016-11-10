pragma solidity ^0.4.2;
import 'common/Mortal.sol';
import 'token/Token.sol';

/**
 * @title Builder v2 contract
 */
contract Builder2 is Mortal {
    /**
     * @dev this event emitted for every builded contract
     */
    event Builded(address indexed sender, address indexed instance);
 
    /* Addresses builded contracts at sender */
    mapping(address => address[]) public getContractsOf;
 /**
     * @dev Get last address
     * @return last address contract
     */
    function getLastContract() constant returns (address) {
        var sender_contracts = getContractsOf[msg.sender];
        return sender_contracts[sender_contracts.length - 1];
    }

    /* Building beneficiary */
    address public beneficiary;

    /**
     * @dev Set beneficiary
     * @param _beneficiary is address of beneficiary
     */
    function setBeneficiary(address _beneficiary) onlyOwner
    { beneficiary = _beneficiary; }

    /* Building cost  */
    uint public buildingCostWei;

    /**
     * @dev Set building cost
     * @param _buildingCostWei is cost
     */
    function setCost(uint _buildingCostWei) onlyOwner
    { buildingCostWei = _buildingCostWei; }

    /* Security check report */
    string public securityCheckURI;

    /**
     * @dev Set security check report URI
     * @param _uri is an URI to report
     */
    function setSecurityCheck(string _uri) onlyOwner
    { securityCheckURI = _uri; }

    /* Ledger for payments */
    Token public credit;

    /**
     * @dev Set ledger for payments
     * @param _credit is address of ledger
     */
    function setCreditLedger(Token _credit) onlyOwner
    { credit = _credit; }
}
