pragma solidity ^0.4.4;

import 'contracts/Comission.sol';

library CreatorComission {
    function create(address _ledger, bytes32 _taxman, uint256 _taxPerc) returns (Comission)
    { return new Comission(_ledger, _taxman, _taxPerc); }

    function version() constant returns (string)
    { return "v0.5.0 (a9ea4c6c)"; }

    function abi() constant returns (string)
    { return '[{"constant":false,"inputs":[{"name":"_destination","type":"bytes32"}],"name":"process","outputs":[{"name":"","type":"bool"}],"payable":true,"type":"function"},{"constant":false,"inputs":[],"name":"kill","outputs":[],"payable":false,"type":"function"},{"constant":true,"inputs":[],"name":"taxman","outputs":[{"name":"","type":"bytes32"}],"payable":false,"type":"function"},{"constant":true,"inputs":[],"name":"ledger","outputs":[{"name":"","type":"address"}],"payable":false,"type":"function"},{"constant":false,"inputs":[{"name":"_owner","type":"address"}],"name":"delegate","outputs":[],"payable":false,"type":"function"},{"constant":true,"inputs":[],"name":"owner","outputs":[{"name":"","type":"address"}],"payable":false,"type":"function"},{"constant":true,"inputs":[],"name":"taxPerc","outputs":[{"name":"","type":"uint256"}],"payable":false,"type":"function"},{"inputs":[{"name":"_ledger","type":"address"},{"name":"_taxman","type":"bytes32"},{"name":"_taxPerc","type":"uint256"}],"type":"constructor"}]'; }
}
