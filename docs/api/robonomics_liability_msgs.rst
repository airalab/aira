Robonomics Liability Messages
=============================

Liability.msg
-------------

============== =========================== ==================================================
 Field                 Type                                Description
============== =========================== ==================================================
address        std_msgs/String               The Liability's address
model          robonomics_msgs/Multihash     CPS behavioral model Identifier
objective      robonomics_msgs/Multihash     CPS behavioral model parameters in rosbag file
result         std_msgs/String               Liability result hash
promisee       std_msgs/String               The promisee address
promisor       std_msgs/String               The promisor address (usually CPS)
token          std_msgs/String               Operational token address
cost           std_msgs/Uint64               CPS behavioral model implementation cost
validator      std_msgs/String               Observing network address
validatorFee   std_msgs/Uint64               Observing network commission
============== =========================== ==================================================

Multihash.msg
-------------

============== =========================== ==========================================
 Field                 Type                                Description
============== =========================== ==========================================
multihash      std_msgs/String              A wrapper for model and objective fields
============== =========================== ==========================================

StartLiability.srv
------------------

**Request**

=========== =============== ===================================================
Field       Type            Description
=========== =============== ===================================================
address     std_msgs/String The address of Liability you are willing to execute
=========== =============== ===================================================

**Response**

=========== =============== ===================================================
Field       Type            Description
=========== =============== ===================================================
success     std_msgs/Bool    Weather or not the Liability was started
msg         std_msgs/String  Status of launch
=========== =============== ===================================================

FinishLiability.srv
-------------------

**Request**

=========== =============== ===================================================
Field       Type            Description
=========== =============== ===================================================
address     std_msgs/String  The address of Liability to finish
success     std_msgs/Bool    The status of execution
=========== =============== ===================================================

**Response**

The response is empty
