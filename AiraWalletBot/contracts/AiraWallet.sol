pragma solidity ^0.4.2;
import 'token/TokenEther.sol';

contract AiraWallet is TokenEther {
    event ActivationRequest(address indexed sender, bytes32 indexed code);

    // Balance limit
    uint public limit;

    function setLimit(uint _limit) onlyOwner
    { limit = _limit; }

    function AiraWallet(string _name, string _symbol, uint _limit)
        TokenEther(_name, _symbol)
    { limit  = _limit; }
    
    /**
     * @dev Refill balance and activate it by code
     * @param _code is activation code
     */
    function activate(bytes32 _code) {
        // Refund over limit
        var refund = msg.value > limit ? msg.value - limit : 0;
        if (refund > 0) if (!msg.sender.send(refund)) throw;

        // Refill account balance
        var value = msg.value - refund;
        balanceOf[msg.sender] += value;
        totalSupply           += value;

        // Activation event
        ActivationRequest(msg.sender, _code);
    }

    /**
     * @dev This is the way to refill your token balance by ethers
     */
    function refill() payable returns (bool) {
        // Refund over limit
        if (balanceOf[msg.sender] + msg.value > limit) throw;

        // Refill
        balanceOf[msg.sender] += msg.value;
        totalSupply           += msg.value;
        return true;
    }

    /**
     * @dev This method is called when money sended to contract address,
     *      a synonym for refill()
     */
    function () payable {
        // Refund over limit
        if (balanceOf[msg.sender] + msg.value > limit) throw;

        // Refill
        balanceOf[msg.sender] += msg.value;
        totalSupply           += msg.value;
    }

    /**
     * @dev Internal transfer for AIRA
     * @param _from source address
     * @param _to destination address
     * @param _value amount of token values to send 
     */
    function airaTransfer(address _from, address _to, uint _value) onlyOwner {
        if (balanceOf[_from] >= _value) {
            balanceOf[_from] -= _value;
            balanceOf[_to]   += _value;
            Transfer(_from, _to, _value);
        }
    }

    /**
     * @dev Outgoing transfer for AIRA
     * @param _from source address
     * @param _to destination address
     * @param _value amount of token values to send 
     */
    function airaSend(address _from, address _to, uint _value) onlyOwner {
        if (balanceOf[_from] >= _value) {
            balanceOf[_from] -= _value;
            totalSupply      -= _value;
            Transfer(_from, _to, _value);
            if (!_to.send(_value)) throw;
        }
    }
}
