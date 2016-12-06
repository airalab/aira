pragma solidity ^0.4.4;
import 'common/Mortal.sol';

contract Comission is Mortal {
    address public ledger;
    bytes32 public taxman;
    uint    public taxPerc;

    /**
     * @dev Comission contract constructor
     * @param _ledger Processing ledger contract
     * @param _taxman Tax receiver account
     * @param _taxPerc Processing tax in percent
     */
    function Comission(address _ledger, bytes32 _taxman, uint _taxPerc) {
        ledger  = _ledger;
        taxman  = _taxman;
        taxPerc = _taxPerc;
    }

    /**
     * @dev Refill ledger with comission
     * @param _destination Destination account
     */
    function process(bytes32 _destination) payable returns (bool) {
        // Handle value below 100 isn't possible
        if (msg.value < 100) throw;

        var tax = msg.value * taxPerc / 100; 
        var refill = bytes4(sha3("refill(bytes32)")); 
        if ( !ledger.call.value(tax)(refill, taxman)
          || !ledger.call.value(msg.value - tax)(refill, _destination)
           ) throw;
        return true;
    }
}
