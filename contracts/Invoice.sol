pragma solidity ^0.4.4;
import 'common/Mortal.sol';
import './Comission.sol';

contract Invoice is Mortal {
    address   public signer;
    uint      public closeBlock;

    Comission public comission;
    string    public description;
    bytes32   public beneficiary;
    uint      public value;

    /**
     * @dev Offer type contract
     * @param _comission Comission handler address
     * @param _description Deal description
     * @param _beneficiary Beneficiary account
     * @param _value Deal value
     */
    function Invoice(address _comission,
                     string  _description,
                     bytes32 _beneficiary,
                     uint    _value) {
        comission   = Comission(_comission);
        description = _description;
        beneficiary = _beneficiary;
        value       = _value;
    }

    /**
     * @dev Call me to withdraw money
     */
    function withdraw() onlyOwner {
        if (closeBlock != 0) {
            if (!comission.process.value(value)(beneficiary)) throw;
        }
    }

    /**
     * @dev Payment fallback function
     */
    function () payable {
        // Condition check
        if (msg.value != value
           || closeBlock != 0) throw;

        // Store block when closed
        closeBlock = block.number;
        signer = msg.sender;
        PaymentReceived();
    }
    
    /**
     * @dev Payment notification
     */
    event PaymentReceived();
}
