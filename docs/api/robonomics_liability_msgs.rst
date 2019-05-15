Robonomics Liability Messages
=============================

.. _Robonomics-Liability-Liability.msg:

robonomics_liability/Liability.msg
----------------------------------

============== ============================================================ ==================================================
 Field                 Type                                                 Description
============== ============================================================ ==================================================
address        :ref:`ethereum_common/Address <Ethereum-common-Address.msg>` The Liability's address
model          :ref:`ipfs_common/Multihash <IPFS-Common-Multihash.msg>`     CPS behavioral model Identifier
objective      :ref:`ipfs_common/Multihash <IPFS-Common-Multihash.msg>`     CPS behavioral model parameters in rosbag file
result         :ref:`ipfs_common/Multihash <IPFS-Common-Multihash.msg>`     Liability result hash
promisee       :ref:`ethereum_common/Address <Ethereum-common-Address.msg>` The promisee address
promisor       :ref:`ethereum_common/Address <Ethereum-common-Address.msg>` The promisor address (usually CPS)
lighthouse     :ref:`ethereum_common/Address <Ethereum-common-Address.msg>` The address of lighthouse your CPS works on
token          :ref:`ethereum_common/Address <Ethereum-common-Address.msg>` Operational token address
cost           :ref:`ethereum_common/UInt256 <Ethereum-common-UInt256.msg>` CPS behavioral model implementation cost
validator      :ref:`ethereum_common/Address <Ethereum-common-Address.msg>` Observing network address
validatorFee   :ref:`ethereum_common/UInt256 <Ethereum-common-UInt256.msg>` Observing network commission
============== ============================================================ ==================================================

.. _IPFS-Common-Multihash.msg:

ipfs_common/Multihash.msg
-------------------------

============== ============================================================ ==================================================
 Field                 Type                                                 Description
============== ============================================================ ==================================================
multihash      std_msgs/String                                              A wrapper for model and objective fields
============== ============================================================ ==================================================

robonomics_liability/StartLiability.srv
---------------------------------------

**Request**

============== ========================================================== ====================================================
Field               Type                                                    Description
============== ========================================================== ====================================================
address             std_msgs/String                                       The address of Liability you are willing to execute
============== ========================================================== ====================================================

**Response**

============== ============================================================ ==================================================
Field               Type                                                       Description
============== ============================================================ ==================================================
success         std_msgs/Bool                                                Weather or not the Liability was started
msg             std_msgs/String                                              Status of launch
============== ============================================================ ==================================================

robonomics_liability/FinishLiability.srv
----------------------------------------

**Request**

============== ============================================================ ==================================================
Field           Type                                                         Description
============== ============================================================ ==================================================
address         std_msgs/String                                              The address of Liability to finish
success         std_msgs/Bool                                                The status of execution
============== ============================================================ ==================================================

**Response**

The response is empty
