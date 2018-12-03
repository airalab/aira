Ethereum Common Messages
========================

Address.msg
-----------

=============== =============== ==============================
Field           Type            Description
=============== =============== ==============================
address         std_msgs/String Address in Ethereum blockchain
=============== =============== ==============================

UInt256.msg
-----------

=============== =============== ==========================
Field           Type            Description
=============== =============== ==========================
uint256         std_msgs/String A wrapper for big integers
=============== =============== ==========================


TransferEvent.msg
-----------------

=============== ======================= ================
Field           Type                    Description
=============== ======================= ================
args_from       ethereum_common/Address Sender address
args_to         ethereum_common/Address Receiver address
args_value      ethereum_common/Uint256 Amount of tokens
=============== ======================= ================

ApprovalEvent.msg
-----------------

=============== ======================= ================
Field           Type                    Description
=============== ======================= ================
args_owner      ethereum_common/Address Owner address
args_spender    ethereum_common/Address Spender address
args_value      ethereum_common/UInt256 Amount of tokens
=============== ======================= ================

.. _Ethereum-common-AccountBalance.srv:

AccountBalance.srv
------------------

**Request**

=========== ======================= ===================================================
Field       Type                    Description
=========== ======================= ===================================================
account     ethereum_common/Address Ethereum address
=========== ======================= ===================================================

**Response**

=========== ======================= ===================================================
Field       Type                    Description
=========== ======================= ===================================================
balance     ethereum_common/UInt256 Balance in Wei
=========== ======================= ===================================================

.. _Ethereum-common-AccountToAddressAllowance.srv:

AccountToAddressAllowance.srv
------------------

**Request**

=========== ======================= ===================================================
Field       Type                    Description
=========== ======================= ===================================================
account     ethereum_common/Address Ethereum address
to          ethereum_common/Address Ethereum address
=========== ======================= ===================================================

**Response**

=========== ======================= ===================================================
Field       Type                    Description
=========== ======================= ===================================================
amount      ethereum_common/UInt256 Balance in Wn
=========== ======================= ===================================================

.. _Ethereum-common-Accounts.srv:

Accounts.srv
------------------

**Request**

Request is empty

**Response**

=========== =========================== ===================================================
Field       Type                        Description
=========== =========================== ===================================================
accounts    ethereum_common/Address[]   List of available accounts
=========== =========================== ===================================================

.. _Ethereum-common-Allowance.srv:

Allowance.srv
------------------

**Request**

Request is empty

**Response**

=========== =========================== ===================================================
Field       Type                        Description
=========== =========================== ===================================================
amount      ethereum_common/UInt256     Amount of XRT the Factory is allowed to spend
=========== =========================== ===================================================

.. _Ethereum-common-Approve.srv:

Approve.srv
-----------

**Request**

=========== =========================== ===================================================
Field       Type                        Description
=========== =========================== ===================================================
spender     ethereum_common/Address     Who is allowed to spend
value       ethereum_common/UInt256     How much tokens are allowed
=========== =========================== ===================================================

**Response**

=========== =========================== ===================================================
Field       Type                        Description
=========== =========================== ===================================================
txhash      std_msgs/Uint8[32]          Transaction hash
=========== =========================== ===================================================

.. _Ethereum-common-Balance.srv:

Balance.srv
-----------

**Request**

Request is empty

**Response**

=========== =========================== ===================================================
Field       Type                        Description
=========== =========================== ===================================================
balance     ethereum_common/UInt256     The balance of default account
=========== =========================== ===================================================

.. _Ethereum-common-BlockNumber.srv:

BlockNumber.srv
---------------

**Request**

Request is empty

**Response**

=========== =========================== ===================================================
Field       Type                        Description
=========== =========================== ===================================================
number      std_msgs/Uint64             Current block number
=========== =========================== ===================================================

.. _Ethereum-common-Transfer.srv:

Transfer.srv
------------

**Request**

=========== =========================== ===================================================
Field       Type                        Description
=========== =========================== ===================================================
to          ethereum_common/Address     Ethereum address
value       ethereum_common/UInt256     The amount of tokens
=========== =========================== ===================================================

**Response**

=========== =========================== ===================================================
Field       Type                        Description
=========== =========================== ===================================================
txhash      std_msgs/Uint8[32]          Transaction hash
=========== =========================== ===================================================

.. _Ethereum-common-TransferFrom.srv:

TransferFrom.srv
----------------

**Request**

=========== =========================== ===================================================
Field       Type                        Description
=========== =========================== ===================================================
owner       ethereum_common/Address     Owner's address
to          ethereum_common/Address     Another account
value       ethereum_common/UInt256     The amount of tokens
=========== =========================== ===================================================

**Response**

=========== =========================== ===================================================
Field       Type                        Description
=========== =========================== ===================================================
txhash      std_msgs/Uint8[32]          Transaction hash
=========== =========================== ===================================================
