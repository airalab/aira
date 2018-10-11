Robonomics Network: How it works
================================

In this section we will discuss the Robonomics Network scenario. 

Robonomics Network uses IPFS PubSub channels for messages exchanging. There are three types of messages: Demand, Offer, Result.

**Below is the specification for a Demand message:**

+--------------+---------+-------------------------------------------------------+------------------------------------------------+
|    Field     |  Type   |                      Description                      |                    Example                     |
+==============+=========+=======================================================+================================================+
| model        | string  | Identifier of the CPS behavioral model                | QmfXHZ2YkNC5vRjp1oAaRoDHD8H3zZznfhBPasTu348eWC |
+--------------+---------+-------------------------------------------------------+------------------------------------------------+
| objective    | string  | Parameters of the CPS behavioral model in rosbag file | QmUo3vvSXZPQaQWjb3cH3qQo1hc8vAUqNnqbdVABbSLb6r |
+--------------+---------+-------------------------------------------------------+------------------------------------------------+
| token        | string  | Address of operational token                          | 0xbD949595eE52346c225a19724084cE517B2cB735     |
+--------------+---------+-------------------------------------------------------+------------------------------------------------+
| cost         | uint32  | Cost of the CPS behavioral model implementation       | 1                                              |
+--------------+---------+-------------------------------------------------------+------------------------------------------------+
| validator    | string  | Observing network address                             | 0x0000000000000000000000000000000000000000     |
+--------------+---------+-------------------------------------------------------+------------------------------------------------+
| validatorFee | uint32  | Observing network commission                          | 0                                              |
+--------------+---------+-------------------------------------------------------+------------------------------------------------+
| deadline     | uint32  | Deadline block number                                 | 6393332                                        |
+--------------+---------+-------------------------------------------------------+------------------------------------------------+
| nonce        | uint8[] | Random data                                           | 0x8e0c...55cb                                  |
+--------------+---------+-------------------------------------------------------+------------------------------------------------+
| signature    | uint8[] | Sender's digital signature                            | 0x23bc...c617                                  |
+--------------+---------+-------------------------------------------------------+------------------------------------------------+

An Offer message has the same fields except instead of ``validator`` and ``validatorFee`` is has ``lighthouseFee`` field. The field tells how much fee for a lighthouse is.

Now let's have a look at the following diagram and walk step by step from the moment of where we publish messages and to where a liability is finalized.

.. image:: ../img/5.png
   :alt: Scenario
   :align: center

A liability contract is created only if the following fields match: ``model``, ``objective``, ``token``, ``cost``. A provider of Robonomics Network watches every message and finds the matched ones. 
When a provider finds corresponding messages it calls ``createLiability(demand, offer)`` method from the contract factory. ``demand`` and ``offer`` are serialized. 

The factory deserializes arguments and recovers a *promisee* and *promisor* addresses from signatures. 

Next step is token transfer. The factory transfers **cost** tokens from *promisee* address and **validatorFee** and **lighthouseFee** from *promisor* address to a new liability address.

.. note::

    You should approve sufficient amount of tokens for the factory.

.. note::
    
    It's not required to approve tokens from *promisor* address if fees are null.    

Now the factory emits NewLiability event with the liability address. An agent gets the address, reads fields and perform a task. During the work the agent writes log file in rosbag format.

When the work is done an agent sends a Result message with the following fields: hash of rosbag file, success flag, signature. If **validator** is not null then only validator is able to finalize a liability.

After a liability finalization and if it's successful an agent gets **cost** tokens otherwise *promisee* gets tokens back.
