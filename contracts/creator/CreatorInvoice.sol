pragma solidity ^0.4.4;

import 'contracts/Invoice.sol';

library CreatorInvoice {
    function create(address _comission, string _description, bytes32 _beneficiary, uint256 _value) returns (Invoice)
    { return new Invoice(_comission, _description, _beneficiary, _value); }

    function version() constant returns (string)
    { return "v0.5.0 (a9ea4c6c)"; }

    function abi() constant returns (string)
    { return '[{"constant":true,"inputs":[],"name":"signer","outputs":[{"name":"","type":"address"}],"payable":false,"type":"function"},{"constant":true,"inputs":[],"name":"beneficiary","outputs":[{"name":"","type":"bytes32"}],"payable":false,"type":"function"},{"constant":true,"inputs":[],"name":"comission","outputs":[{"name":"","type":"address"}],"payable":false,"type":"function"},{"constant":false,"inputs":[],"name":"withdraw","outputs":[],"payable":false,"type":"function"},{"constant":true,"inputs":[],"name":"value","outputs":[{"name":"","type":"uint256"}],"payable":false,"type":"function"},{"constant":false,"inputs":[],"name":"kill","outputs":[],"payable":false,"type":"function"},{"constant":false,"inputs":[{"name":"_owner","type":"address"}],"name":"delegate","outputs":[],"payable":false,"type":"function"},{"constant":true,"inputs":[],"name":"description","outputs":[{"name":"","type":"string"}],"payable":false,"type":"function"},{"constant":true,"inputs":[],"name":"closeBlock","outputs":[{"name":"","type":"uint256"}],"payable":false,"type":"function"},{"constant":true,"inputs":[],"name":"owner","outputs":[{"name":"","type":"address"}],"payable":false,"type":"function"},{"inputs":[{"name":"_comission","type":"address"},{"name":"_description","type":"string"},{"name":"_beneficiary","type":"bytes32"},{"name":"_value","type":"uint256"}],"type":"constructor"},{"payable":true,"type":"fallback"},{"anonymous":false,"inputs":[],"name":"PaymentReceived","type":"event"}]'; }
}
