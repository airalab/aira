pragma solidity ^0.4.2;
import 'token/TokenHash.sol';
import './AiraRegistrarService.sol';

contract AiraEtherFunds is TokenHash {
    function AiraEtherFunds(address _bot_reg, string _name, string _symbol)
            TokenHash(_name, _symbol, 18, 0) {
        reg = AiraRegistrarService(_bot_reg);
    }

    /**
     * @dev Event spawned when activation request received
     */
    event ActivationRequest(address indexed ident, bytes32 indexed code);

    // Balance limit
    uint public limit;
    
    function setLimit(uint _limit) onlyOwner
    { limit = _limit; }

    // Account activation fee
    uint public fee;
    
    function setFee(uint _fee) onlyOwner
    { fee = _fee; }

    /**
     * @dev Refill balance and activate it by code
     * @param _code is activation code
     */
    function activate(string _code) payable {
        var value = msg.value;
 
        // Get a fee
        if (fee > 0) {
            if (value < fee) throw;
            balanceOf[sha3(owner)] += fee;
            value                  -= fee;
        }

        // Refund over limit
        if (limit > 0 && value > limit) {
            var refund = value - limit;
            if (!msg.sender.send(refund)) throw;
            value = limit;
        }

        // Refill account balance
        balanceOf[sha3(msg.sender)] += value;
        totalSupply                 += value;

        // Activation event
        ActivationRequest(msg.sender, stringToBytes32(_code));
    }

    /**
     * @dev String to bytes32 conversion helper
     */
    function stringToBytes32(string memory source) constant returns (bytes32 result)
    { assembly { result := mload(add(source, 32)) } }

    /**
     * @dev This is the way to refill token balance by ethers
     * @param _dest is destination address
     */
    function refill(address _dest) payable returns (bool)
    { return refill(sha3(_dest)); }

    /**
     * @dev This method is called when money sended to contract address,
     *      a synonym for refill()
     */
    function () payable
    { refill(sha3(msg.sender)); }

    /**
     * @dev This is the way to refill token balance by ethers
     * @param _dest is destination identifier
     */
    function refill(bytes32 _dest) payable returns (bool) {
        // Throw when over limit
        if (balanceOf[_dest] + msg.value > limit) throw;

        // Refill
        balanceOf[_dest] += msg.value;
        totalSupply      += msg.value;
        return true;
    }

    /**
     * @dev Outgoing transfer (send) with allowance
     * @param _from source identifier
     * @param _to external destination address
     * @param _value amount of token values to send 
     */
    function sendFrom(bytes32 _from, address _to, uint _value) {
        var sender = sha3(msg.sender);
        var avail = allowance[_from][sender]
                  > balanceOf[_from] ? balanceOf[_from]
                                     : allowance[_from][sender];
        if (avail >= _value) {
            allowance[_from][sender] -= _value;
            balanceOf[_from]         -= _value;
            totalSupply              -= _value;
            if (!_to.send(_value)) throw;
        }
    }

    AiraRegistrarService public reg;
    modifier onlySecure { if (msg.sender != reg.addr("AiraSecure")) throw; _; }

    /**
     * @dev Increase approved token values for AiraEthBot
     * @param _client is a client ident
     * @param _value is amount of tokens
     */
    function secureApprove(bytes32 _client, uint _value) onlySecure {
        var ethBot = reg.addr("AiraEth");
        if (ethBot != 0)
            allowance[_client][sha3(ethBot)] += _value;
    }

    /**
     * @dev Close allowance for AiraEthBot
     * @param _client is a client ident
     */
    function secureUnapprove(bytes32 _client) onlySecure {
        var ethBot = reg.addr("AiraEth");
        if (ethBot != 0)
            allowance[_client][sha3(ethBot)] = 0;
    }

    // By security issues, deny to kill this by owner
    function kill() onlyOwner { throw; }
}
