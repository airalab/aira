import 'common/Mortal.sol';

contract Proxy is Owned {
    struct Call {
        address target;
        uint    value;
        bytes   transaction;
        uint    execBlock;
    }
    Call queue;

    /**
     * @dev Get call info by index
     */
    function callAt(uint _index) constant returns (address, uint, bytes, uint) {
        var memory c = queue[_index];
        return (c.target, c.value, c.transaction, c.execBlock);
    }

    /**
     * @dev Get call queue length
     */
    function queueLen() constant returns (uint)
    { return queue.length; }

    bytes32 public client;
    address public sndFactor;

    /**
     * @dev change 2factor address
     */
    function setSndFactor(address _snd_factor) onlySndFactor
    { sndFactor = _snd_factor; }
    
    function Proxy(bytes32 _client, address _snd_factor) {
        client    = _client;
        sndFactor = _snd_factor;
    }

    /**
     * @dev Transaction request
     * @param _target Transaction destination
     * @param _value Transaction value in wei
     * @param _transaction Transaction data
     */
    function request(address _target, uint _value, bytes _transaction) onlyOwner {
        queue.push(Call(_target, _value, _transaction, 0));
        RequestedCall(queue.length - 1);
    }

    /**
     * @dev Call request log
     * @param index Position in call queue
     */
    event CallRequest(uint indexed index);

    /**
     * @dev 2Factor authorization of transaction
     * @param _index Call in queue position
     */
    function authorize(uint _index) onlySndFactor {
        var memory c = queue[_index];
        // Guard
        if (c.target == 0 || c.execBlock != 0) throw;
        // Store exec block
        queue[_index].execBlock = block.number;
        // Run transaction
        c.target.call.value(c.value)(c.transaction);
    }
}
