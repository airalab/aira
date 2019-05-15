Market Messages
===============

Market messages is used for exchange **Demand** and **Offer** information. It also used for delivery **Result** messages with liability execution reports.

.. note::

   This is spec for Robonomics ``Generation 5``.

* Currently for message delivery is used IPFS PubSub_ broadcaster.
* IPFS PubSub **topic** is set according to *Lighthouse* ENS_ name.

.. _PubSub: https://ipfs.io/blog/25-pubsub/
.. _ENS: https://ens.domains/

Messages content
----------------

Robonomics market message use JSON_ data format.

.. _JSON: https://www.json.org/

**Demand**

 ============== ============================================================== ================================================
      Field                              ROS Type                                                Description
 ============== ============================================================== ================================================
  model          :ref:`ipfs_common/Multihash <IPFS-Common-Multihash.msg>`       CPS behavioral model identifier
  objective      :ref:`ipfs_common/Multihash <IPFS-Common-Multihash.msg>`       CPS behavioral model parameters in rosbag file
  token          :ref:`ethereum_common/Address <Ethereum-common-Address.msg>`   Operational token address
  cost           :ref:`ethereum_common/UInt256 <Ethereum-common-UInt256.msg>`   CPS behavioral model execution cost
  lighthouse     :ref:`ethereum_common/Address <Ethereum-common-Address.msg>`   Lighthouse contract address
  validator      :ref:`ethereum_common/Address <Ethereum-common-Address.msg>`   Observing network address
  validatorFee   :ref:`ethereum_common/UInt256 <Ethereum-common-UInt256.msg>`   Observing network fee 
  deadline       :ref:`ethereum_common/UInt256 <Ethereum-common-UInt256.msg>`   Deadline block number
  sender         :ref:`ethereum_common/Address <Ethereum-common-Address.msg>`   Message sender address
  signature      std_msgs/UInt8[]                                               Sender's Ethereum signature
 ============== ============================================================== ================================================

**Offer**

 =============== ============================================================== ================================================
      Field                              ROS Type                                                Description
 =============== ============================================================== ================================================
  model           :ref:`ipfs_common/Multihash <IPFS-Common-Multihash.msg>`       CPS behavioral model identifier
  objective       :ref:`ipfs_common/Multihash <IPFS-Common-Multihash.msg>`       CPS behavioral model parameters in rosbag file
  token           :ref:`ethereum_common/Address <Ethereum-common-Address.msg>`   Operational token address
  cost            :ref:`ethereum_common/UInt256 <Ethereum-common-UInt256.msg>`   CPS behavioral model execution cost
  validator       :ref:`ethereum_common/Address <Ethereum-common-Address.msg>`   Observing network address
  lighthouse      :ref:`ethereum_common/Address <Ethereum-common-Address.msg>`   Lighthouse contract address
  lighthouseFee   :ref:`ethereum_common/UInt256 <Ethereum-common-UInt256.msg>`   Liability creation fee 
  deadline        :ref:`ethereum_common/UInt256 <Ethereum-common-UInt256.msg>`   Deadline block number
  sender          :ref:`ethereum_common/Address <Ethereum-common-Address.msg>`   Message sender address
  signature       std_msgs/UInt8[]                                               Sender's Ethereum signature
 =============== ============================================================== ================================================

**Result**

 =========== ============================================================== ===========================================
    Field                                 ROS Type                                             Description
 =========== ============================================================== ===========================================
  liability   :ref:`ethereum_common/Address <Ethereum-common-Address.msg>`   Liability contract address
  result      :ref:`ipfs_common/Multihash <IPFS-Common-Multihash.msg>`       Liability result multihash
  success     std_msgs/Bool                                                  Is liability executed successful
  signature   std_msgs/UInt8[]                                               Sender's Ethereum signature
 =========== ============================================================== ===========================================

Messages signing
----------------

Before signing the messages is packed using abi.encodePacked_ solidity finction and hashed by Keccak_256.

.. _abi.encodePacked: https://solidity.readthedocs.io/en/latest/abi-spec.html#non-standard-packed-mode

.. code-block:: solidity

   demandHash = keccak256(abi.encodePacked(
        _model
      , _objective
      , _token
      , _cost
      , _lighthouse
      , _validator
      , _validator_fee
      , _deadline
      , IFactory(factory).nonceOf(_sender)
      , _sender
      ));

.. note::

   ``nonce`` parameter is counted by factory smart contract and incremented for each created liability smart contract.

Message hash are signed using Ethereum ``secp256k1`` signature_.

.. _signature: https://github.com/ethereum/wiki/wiki/JSON-RPC#eth_sign
