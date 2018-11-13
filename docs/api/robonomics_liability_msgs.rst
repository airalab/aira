Robonomics Liability Messages
=============================

Liability.msg
-------------

=============== =========== ==============================================
Field           Type        Description
=============== =========== ==============================================
address         string      The Liability's address
model           string      CPS behavioral model Identifier
objective       string      CPS behavioral model parameters in rosbag file
result          string      Liability result hash
promisee        string      The promisee address
promisor        string      The promisor address (usually CPS)
token           string      Operational token address
cost            uint64      CPS behavioral model implementation cost
validator       string      Observing network address
validatorFee    uint64      Observing network commission
=============== =========== ==============================================

StartLiability.srv
------------------

**Request**

=========== ======= ===================================================
Field       Type    Description
=========== ======= ===================================================
address     string  The address of Liability you are willing to execute
=========== ======= ===================================================

**Response**

=========== ======= ===================================================
Field       Type    Description
=========== ======= ===================================================
success     bool    Weather or not the Liability was started
msg         string  Status of launch
=========== ======= ===================================================

FinishLiability.srv
-------------------

**Request**

=========== ======= ===================================================
Field       Type    Description
=========== ======= ===================================================
address     string  The address of Liability to finish
success     bool    The status of execution
=========== ======= ===================================================

**Response**

The response is empty
