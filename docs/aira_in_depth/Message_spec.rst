Robonomics messages
===================

.. note::

   This is Robonomics network ``Generation 3`` message specification.

* Currently for message delivery is used IPFS PubSub_ broadcaster.
* IPFS PubSub **topic** is set according to *Lighthouse* ENS_ name.
* Robonomics message sended serialized by JSON_.

.. _PubSub: https://ipfs.io/blog/25-pubsub/
.. _ENS: https://ens.domains/
.. _JSON: https://www.json.org/

Specification
-------------

**Demand**

 ===============  =========  ================================================
  Field            Type       Description
 ===============  =========  ================================================
  model            string     CPS behavioral model Identifier
  objective        string     CPS behavioral model parameters in rosbag file
  token            string     Operational token address
  cost             ineger     CPS behavioral model implementation cost
  lighthouse       string     Lighthouse address
  validator        string     Observing network address
  validatorFee     integer    Observing network comission
  deadline         integer    Deadline block number
  nonce            array      Random uniq data
  signature        array      Sender's digital signature
 ===============  =========  ================================================

**Offer**

 ===============  =========  ================================================
  Field            Type       Description
 ===============  =========  ================================================
  model            string     CPS behavioral model Identifier
  objective        string     CPS behavioral model parameters in rosbag file
  token            string     Operational token address
  cost             ineger     CPS behavioral model implementation cost
  validator        string     Observing network address
  lighthouse       string     Lighthouse address 
  lighthouseFee    integer    Liability creation commission
  deadline         integer    Deadline block number
  nonce            array      Random uniq data
  signature        array      Sender's digital signature
 ===============  =========  ================================================

**Result**

 ============  ==========    ================================================
  Field         Type          Description
 ============  ==========    ================================================
  liability     string        Liability contract address
  result        string        Liability result hash encoded as Base58
  success       boolean       Is liability executed successful
  signature     array         Sender's digital signature
 ============  ==========    ================================================

.. _Type: JSON type
