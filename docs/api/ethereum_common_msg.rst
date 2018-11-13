Ethereum Common Messages
========================

Address.msg
-----------

=============== =========== ==============================
Field           Type        Description
=============== =========== ==============================
address         string      Address in Ethereum blockchain
=============== =========== ==============================

UInt256.msg
-----------

=============== =========== ==========================
Field           Type        Description
=============== =========== ==========================
uint256         string      A wrapper for big integers
=============== =========== ==========================


TransferEvent.msg
-----------------

=============== =========== ================
Field           Type        Description
=============== =========== ================
args_from       Address     Sender address
args_to         address     Receiver address
args_value      uint256     Amount of tokens
=============== =========== ================

ApprovalEvent.msg
-----------------

=============== =========== ================
Field           Type        Description
=============== =========== ================
args_owner      Address     Owner address
args_spender    Address     Spender address
args_value      UInt256     Amount of tokens
=============== =========== ================

.. _Ethereum-common-AccountBalance.srv:

AccountBalance.srv
------------------

**Request**

=========== ======= ===================================================
Field       Type    Description
=========== ======= ===================================================
account     Address Ethereum address
=========== ======= ===================================================

**Response**

=========== ======= ===================================================
Field       Type    Description
=========== ======= ===================================================
balance     UInt256 Balance in Wei
=========== ======= ===================================================

.. _Ethereum-common-AccountToAddressAllowance.srv:

AccountToAddressAllowance.srv
------------------

**Request**

=========== ======= ===================================================
Field       Type    Description
=========== ======= ===================================================
account     Address Ethereum address
to          Address Ethereum address
=========== ======= ===================================================

**Response**

=========== ======= ===================================================
Field       Type    Description
=========== ======= ===================================================
amount      UInt256 Balance in Wn
=========== ======= ===================================================

.. _Ethereum-common-Accounts.srv:

Accounts.srv
------------------

**Request**

Request is empty

**Response**

=========== =========== ===================================================
Field       Type        Description
=========== =========== ===================================================
accounts    Address[]   List of available accounts
=========== =========== ===================================================

.. _Ethereum-common-Allowance.srv:

Allowance.srv
------------------

**Request**

Request is empty

**Response**

=========== =========== ===================================================
Field       Type        Description
=========== =========== ===================================================
amount      UInt256     Amount of XRT the Factory is allowed to spend
=========== =========== ===================================================

.. _Ethereum-common-Approve.srv:

Approve.srv
-----------

**Request**

=========== =========== ===================================================
Field       Type        Description
=========== =========== ===================================================
spender     Address     Who is allowed to spend
value       UInt256     How much tokens are allowed
=========== =========== ===================================================

**Response**

=========== =========== ===================================================
Field       Type        Description
=========== =========== ===================================================
txhash      uint8[32]   Transaction hash
=========== =========== ===================================================

.. _Ethereum-common-Balance.srv:

Balance.srv
-----------

**Request**

Request is empty

**Response**

=========== =========== ===================================================
Field       Type        Description
=========== =========== ===================================================
balance     UInt256     The balance of default account
=========== =========== ===================================================

.. _Ethereum-common-BlockNumber.srv:

BlockNumber.srv
---------------

**Request**

Request is empty

**Response**

=========== =========== ===================================================
Field       Type        Description
=========== =========== ===================================================
number      uint64      Current block number
=========== =========== ===================================================

.. _Ethereum-common-Transfer.srv:

Transfer.srv
------------

**Request**

=========== =========== ===================================================
Field       Type        Description
=========== =========== ===================================================
to          Address     Ethereum address
value       UInt256     The amount of tokens
=========== =========== ===================================================

**Response**

=========== =========== ===================================================
Field       Type        Description
=========== =========== ===================================================
txhash      uint8[32]   Transaction hash
=========== =========== ===================================================

.. _Ethereum-common-TransferFrom.srv:

TransferFrom.srv
----------------

**Request**

=========== =========== ===================================================
Field       Type        Description
=========== =========== ===================================================
owner       Address     Owner's address
to          Address     Another account
value       UInt256     The amount of tokens
=========== =========== ===================================================

**Response**

=========== =========== ===================================================
Field       Type        Description
=========== =========== ===================================================
txhash      uint8[32]   Transaction hash
=========== =========== ===================================================
