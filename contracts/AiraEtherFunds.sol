pragma solidity ^0.4.2;
import 'token/TokenEther.sol';

contract AiraEtherFunds is TokenEther {
    function AiraEtherFunds(string _name, string _symbol) TokenEther(_name, _symbol) {}

    /**
     * @dev Event spawned when activation request received
     */
    event ActivationRequest(address indexed sender, bytes32 indexed code);

    /**
     * @dev String to bytes32 conversion helper
     */
    function stringToBytes32(string memory source) constant returns (bytes32 result)
    { assembly { result := mload(add(source, 32)) } }

    // Balance limit
    uint public limit;
    
    function setLimit(uint _limit) onlyOwner
    { limit = _limit; }

    // Account activation fee
    uint public fee;
    
    function setFee(uint _fee) onlyOwner
    { fee = _fee; }

    // AiraEtherBot
    address public bot;

    function setBot(address _bot) onlyOwner
    { bot = _bot; }

    modifier onlyBot { if (msg.sender != bot) throw; _; }

    /**
     * @dev Refill balance and activate it by code
     * @param _code is activation code
     */
    function activate(string _code) payable {
        var value = msg.value;
 
        // Get a fee
        if (fee > 0) {
            if (value < fee) throw;
            balanceOf[owner] += fee;
            value            -= fee;
        }

        // Refund over limit
        if (limit > 0 && value > limit) {
            var refund = value - limit;
            if (!msg.sender.send(refund)) throw;
            value = limit;
        }

        // Refill account balance
        balanceOf[msg.sender] += value;
        totalSupply           += value;

        // Activation event
        ActivationRequest(msg.sender, stringToBytes32(_code));
    }

    /**
     * @dev This is the way to refill your token balance by ethers
     */
    function refill() payable returns (bool) {
        // Throw when over limit
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
        // Throw when over limit
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
    function airaTransfer(address _from, address _to, uint _value) onlyBot {
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
    function airaSend(address _from, address _to, uint _value) onlyBot {
        if (balanceOf[_from] >= _value) {
            balanceOf[_from] -= _value;
            totalSupply      -= _value;
            Transfer(_from, _to, _value);
            if (!_to.send(_value)) throw;
        }
    }
}
