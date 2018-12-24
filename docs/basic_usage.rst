Basic Usage
===========

To get familiar with AIRA, let's see what is under the hood.

Once you launch the client several ros nodes will already be on the run. Here's a list of robonomics communication stack nodes::

    $ rosnode list
    /liability/executor
    /liability/infochan/channel
    /liability/infochan/signer
    /liability/listener
    /rosout

* ``/liability/executor`` - gets rosbag file from IPFS and plays it
* ``/liability/infochan/channel`` - is responsible for offer, demand and result messages. It catches messages from the channel and sends signed messages back
* ``/liability/infochan/signer`` - offers services for signing offer, demand and result messages
* ``/liability/listener`` - watches for a new liability contracts. When the event is received the node calls executor node

And here's a list of robonomics stack topics.

.. code-block:: bash

    $ rostopic list
    /liability/complete
    /liability/finalized
    /liability/incoming
    /liability/infochan/eth/sending/demand
    /liability/infochan/eth/sending/offer
    /liability/infochan/eth/sending/result
    /liability/infochan/eth/signing/demand
    /liability/infochan/eth/signing/offer
    /liability/infochan/eth/signing/result
    /liability/infochan/incoming/demand
    /liability/infochan/incoming/offer
    /liability/infochan/incoming/result
    /liability/ready
    /liability/result
    /rosout
    /rosout_agg

The most important topics for us are:

* ``/liability/incoming`` - when a new liability is created, this topic publishes Ethereum address of the contract
* ``/liability/result`` - this topic is for publishing results. But don't publish a result directly to this topic! Use a service instead
* ``/liability/infochan/incoming/*`` - a CPS gets information about offer, demand or result from corresponding topics
* ``/liability/infochan/eth/signing/*`` - a CPS sends offer, demand or result messages to corresponding topics

Let's start with greetings - say hello to AIRA!

You should just launch a pre-installed package ``hello_aira``::

    $ rosrun hello_aira hello_aira

We've launched our agent. It will wait for a demand message. Now it's time to send the message. Go to `dapp <https://airalab.github.io/robonomics_tutorials/>`_ and press Order.
Now go back to the console and see the result!
