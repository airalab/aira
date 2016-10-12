pragma solidity ^0.4.2;
import 'token/TokenEther.sol';
import './OwnedRegistrar.sol';

contract AiraWallet is TokenEther {
    // Registrar
    OwnedRegistrar public reg;

    // Registration fee
    uint public regFee;

    // Balance limit
    uint public limit;

    // Reverse mapping for taking account by address
    mapping(address => string) public reverse;

    function AiraWallet(string _name, string _symbol,
                        uint _reg_fee, uint _limit)
        TokenEther(_name, _symbol)
    { 
        reg    = new OwnedRegistrar();
        regFee = _reg_fee;
        limit  = _limit;
    }

    /**
     * @dev Register new user account and refill balance
     * @param _name is a user name
     */
    function register(string _name) payable returns (bool) {
        if (msg.value < regFee) throw;
        if (reg.addr(_name) != 0) throw;

        // Fee
        balanceOf[owner] += regFee;
        var value = msg.value - regFee;

        // Refund over limit
        var refund = value > limit ? value - limit : 0;
        if (!msg.sender.send(refund)) throw;

        // Refill account balance
        var cleanValue = value - refund;
        balanceOf[msg.sender]        += cleanValue;
        allowance[msg.sender][owner] += cleanValue;
        totalSupply                  += cleanValue;

        // Register account
        reverse[msg.sender] = _name;
        reg.setAddr(_name, msg.sender);
        return true;
    }
 
    /**
     * @dev This is the way to refill your token balance by ethers
     */
    function refill() payable {
        // Refund over limit
        if (balanceOf[msg.sender] + msg.value > limit) throw;

        balanceOf[msg.sender]        += msg.value;
        allowance[msg.sender][owner] += msg.value; 
        totalSupply                  += msg.value;
    }

    /**
     * @dev This method is called when money sended to contract address,
     *      a synonym for refill()
     */
    function () payable {
        // Refund over limit
        if (balanceOf[msg.sender] + msg.value > limit) throw;

        balanceOf[msg.sender]        += msg.value;
        allowance[msg.sender][owner] += msg.value; 
        totalSupply                  += msg.value;
    }

    /**
     * @dev Outgoing transfer with approvement mechainsm
     * @param _from source address, `_value` tokens shold be approved for `sender`
     * @param _to destination address out of token
     * @param _value amount of token values to send 
     * @notice from `_from` will be sended `_value` tokens to `_to`
     * @return `true` when transfer is done
     */
    function transferOut(address _from, address _to, uint _value) returns (bool) {
        var avail = allowance[_from][msg.sender]
                  > balanceOf[_from] ? balanceOf[_from]
                                     : allowance[_from][msg.sender];
        if (avail >= _value) {
            allowance[_from][msg.sender] -= _value;
            balanceOf[_from]             -= _value;
            totalSupply                  -= _value;
            Transfer(_from, _to, _value);
            if (!_to.send(_value)) throw;
            return true;
        }
        return false;
    }
}
