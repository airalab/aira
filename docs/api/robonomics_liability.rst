Robonomics Liability
====================

The package is responsible for receiving `New Liability` events (``listener`` node) and playing topics from `objective` field (``executor`` node).
The launch file also include ``ipfs_channel`` node and ``signer`` node.

ROS Parameters
--------------

.. py:attribute:: ~web3_http_provider

    Web3 HTTP provider address. The type is ``string``, defaults to ``http://127.0.0.1:8545``

.. py:attribute:: ~web3_ws_provider

    Web3 WebSocket provider address. The type is ``string``, defaults to ``ws://127.0.0.1:8546``

.. py:attribute:: ~ipfs_http_provider

    IPFS HTTP provider address. The type is ``string``, defaults to ``http://127.0.0.1:5001``

.. py:attribute:: ~factory_contract

    The name of the liability factory. The type is ``string``, defaults to ``factory.3.robonomics.eth``

.. py:attribute:: ~lighthouse_contract

    The name of a lighthouse you are working on. The type is ``string``, defaults to ``airalab.lighthouse.3.robonomics.eth``

.. py:attribute:: ~enable_executor

    Enable or disable executor node. If it's ``false``, no topics from objective would be published. The type is ``boolean``, defaults to ``true``

.. py:attribute:: ~master_check_interval

    Period (in seconds) to check master for new topic publications. It's necessary for the Recorder, which records all the topics a CPS publishes. The type is ``double``, defaults to ``0.1``

.. py:attribute:: ~recording_topics

    List of topics name separated by comma. It allows you to specify which topics would be recorded. The type is ``string``, defaults to ``""``

.. py:attribute:: ~ens_contract

    The checksumed address of ENS registry. The type is ``string``, defaults to ``""``

.. py:attribute:: ~keyfile

    Path to keyfile. The type is ``string``, defaults to ``""``. **Required parameter**

.. py:attribute:: ~keyfile_password_file

    Path to a file with password for the keyfile. The type is ``string``, defaults to ``""``. **Required parameter**

Subscribed topics
-----------------

.. py:method:: /liability/infochan/eth/signing/demand (robonomics_msgs/Demand)

    `robonomics_msgs/Demand`_ message to sign and send further to IPFS channel

.. py:method:: /liability/infochan/eth/signing/offer (robonomics_msgs/Offer)

    `robonomics_msgs/Offer`_ message to sign and send further to IPFS channel

.. py:method:: /liability/infochan/eth/signing/result (robonomics_msgs/Result)

    `robonomics_msgs/Result`_ message to sign and send further to IPFS channel

.. _robonomics_msgs/Demand: ../aira_in_depth/Message_spec.html
.. _robonomics_msgs/Offer: ../aira_in_depth/Message_spec.html
.. _robonomics_msgs/Result: ../aira_in_depth/Message_spec.html

Published topics
----------------

.. py:method:: /liability/infochan/incoming/demand (robonomics_msgs/Demand)

    Contains a `robonomics_msgs/Demand`_ message which was read from IPFS channel

.. py:method:: /liability/infochan/incoming/offer (robonomics_msgs/Offer)

    Contains a `robonomics_msgs/Offer`_ message which was read from IPFS channel

.. py:method:: /liability/infochan/incoming/result (robonomics_msgs/Result)

    Contains a `robonomics_msgs/Result`_ message which was read from IPFS channel

.. py:method:: /liability/incoming (robonomics_liability/Liability)

    Contains all the information about the last created `robonomics_liability/Liability`_

.. py:method:: /liability/ready (robonomics_liability/Liability)

    Signals when a `robonomics_liability/Liability`_ is ready for execution

.. py:method:: /liability/complete (robonomics_liability/Liability)

    Signals when a `robonomics_liability/Liability`_ has done its job

.. py:method:: /liability/finalized (std_msgs/String)

    Signals when a liability has been finalized

.. _robonomics_liability/Liability: robonomics_liability_msgs.html

Services
--------

.. py:method:: /liability/start (robonomics_liability/StartLiability)

    The service tells executor to play topics from the objective. It's required to pass a liability address (`robonomics_liability/StartLiability`_), which you can get from ``/liability/ready`` topic

.. py:method:: /liability/finish (robonomics_liability/FinishLiability)

    a CPS should call the service after performing the task. Input is `robonomics_liability/FinishLiability`_

.. _robonomics_liability/StartLiability: robonomics_liability_msgs.html
.. _robonomics_liability/FinishLiability: robonomics_liability_msgs.html
