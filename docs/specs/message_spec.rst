Robonomics Messages
===================

.. note::

   This is Robonomics network ``Generation 4`` message specification.

* Currently for message delivery is used IPFS PubSub_ broadcaster.
* IPFS PubSub **topic** is set according to *Lighthouse* ENS_ name.
* Robonomics message sended serialized by JSON_.

.. _PubSub: https://ipfs.io/blog/25-pubsub/
.. _ENS: https://ens.domains/
.. _JSON: https://www.json.org/

Specification
-------------

**Demand**

 ================ ============================================================== ================================================
       Field                                   Type                                                Description
 ================ ============================================================== ================================================
    model          :ref:`ipfs_common/Multihash <IPFS-Common-Multihash.msg>`       CPS behavioral model Identifier
    objective      :ref:`ipfs_common/Multihash <IPFS-Common-Multihash.msg>`       CPS behavioral model parameters in rosbag file
    token          :ref:`ethereum_common/Address <Ethereum-common-Address.msg>`   Operational token address
    cost           :ref:`ethereum_common/UInt256 <Ethereum-common-UInt256.msg>`   CPS behavioral model implementation cost
    lighthouse     :ref:`ethereum_common/Address <Ethereum-common-Address.msg>`   Lighthouse address
    validator      :ref:`ethereum_common/Address <Ethereum-common-Address.msg>`   Observing network address
    validatorFee   :ref:`ethereum_common/UInt256 <Ethereum-common-UInt256.msg>`   Observing network commission
    deadline       :ref:`ethereum_common/UInt256 <Ethereum-common-UInt256.msg>`   Deadline block number
    nonce          std_msgs/UInt8[]                                               Random unique data
    signature      std_msgs/UInt8[]                                               Sender's digital signature
 ================ ============================================================== ================================================

**Offer**

 =============== ============================================================== ================================================
      Field                                   Type                                                Description
 =============== ============================================================== ================================================
  model           :ref:`ipfs_common/Multihash <IPFS-Common-Multihash.msg>`       CPS behavioral model Identifier
  objective       :ref:`ipfs_common/Multihash <IPFS-Common-Multihash.msg>`       CPS behavioral model parameters in rosbag file
  token           :ref:`ethereum_common/Address <Ethereum-common-Address.msg>`   Operational token address
  cost            :ref:`ethereum_common/UInt256 <Ethereum-common-UInt256.msg>`   CPS behavioral model implementation cost
  validator       :ref:`ethereum_common/Address <Ethereum-common-Address.msg>`   Observing network address
  lighthouse      :ref:`ethereum_common/Address <Ethereum-common-Address.msg>`   Lighthouse address
  lighthouseFee   :ref:`ethereum_common/UInt256 <Ethereum-common-UInt256.msg>`   Liability creation commission
  deadline        :ref:`ethereum_common/UInt256 <Ethereum-common-UInt256.msg>`   Deadline block number
  nonce           std_msgs/UInt8[]                                               Random unique data
  signature       std_msgs/UInt8[]                                               Sender's digital signature
 =============== ============================================================== ================================================

**Result**

 =========== ============================================================== =========================================
    Field                                 Type                                             Description
 =========== ============================================================== =========================================
  liability   :ref:`ethereum_common/Address <Ethereum-common-Address.msg>`   Liability contract address
  result      :ref:`ipfs_common/Multihash <IPFS-Common-Multihash.msg>`       Liability result hash encoded as Base58
  success     std_msgs/Bool                                                  Is liability executed successful
  signature   std_msgs/UInt8[]                                               Sender's digital signature
 =========== ============================================================== =========================================

.. _Type: JSON type
