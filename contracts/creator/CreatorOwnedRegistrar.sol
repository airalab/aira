pragma solidity ^0.4.2;

import 'contracts/AiraRegistrarService.sol';

library CreatorOwnedRegistrar {
    function create() returns (AiraRegistrarService)
    { return new AiraRegistrarService(); }

    function version() constant returns (string)
    { return "v0.5.0 (84e7c7f9)"; }

    function abi() constant returns (string)
    { return '[{"constant":false,"inputs":[{"name":"_name","type":"string"}],"name":"disown","outputs":[],"payable":false,"type":"function"},{"constant":false,"inputs":[{"name":"_name","type":"string"},{"name":"_a","type":"address"}],"name":"setAddr","outputs":[],"payable":false,"type":"function"},{"constant":false,"inputs":[],"name":"kill","outputs":[],"payable":false,"type":"function"},{"constant":true,"inputs":[{"name":"_name","type":"string"}],"name":"addr","outputs":[{"name":"","type":"address"}],"payable":false,"type":"function"},{"constant":false,"inputs":[{"name":"_owner","type":"address"}],"name":"delegate","outputs":[],"payable":false,"type":"function"},{"constant":true,"inputs":[{"name":"_name","type":"string"}],"name":"subRegistrar","outputs":[{"name":"","type":"address"}],"payable":false,"type":"function"},{"constant":true,"inputs":[],"name":"owner","outputs":[{"name":"","type":"address"}],"payable":false,"type":"function"},{"constant":false,"inputs":[{"name":"_name","type":"string"},{"name":"_registrar","type":"address"}],"name":"setSubRegistrar","outputs":[],"payable":false,"type":"function"},{"constant":true,"inputs":[{"name":"_name","type":"string"}],"name":"content","outputs":[{"name":"","type":"bytes32"}],"payable":false,"type":"function"},{"constant":true,"inputs":[{"name":"_name","type":"string"}],"name":"owner","outputs":[{"name":"o_owner","type":"address"}],"payable":false,"type":"function"},{"constant":true,"inputs":[{"name":"_name","type":"string"}],"name":"record","outputs":[{"name":"o_addr","type":"address"},{"name":"o_subRegistrar","type":"address"},{"name":"o_content","type":"bytes32"}],"payable":false,"type":"function"},{"constant":false,"inputs":[{"name":"_name","type":"string"},{"name":"_content","type":"bytes32"}],"name":"setContent","outputs":[],"payable":false,"type":"function"},{"anonymous":false,"inputs":[{"indexed":true,"name":"name","type":"string"}],"name":"Changed","type":"event"}]'; }
}
