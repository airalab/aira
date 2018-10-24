Basic Usage
===========

To get familiar with AIRA, let's see what is under the hood. 

Once you launch the client several ros nodes will already be on the run. Here's a list of robonomics communication stack nodes::

    $ rosnode list
    /liability/executor
    /liability/infochan/channel
    /liability/infochan/signer
    /rosout

* ``/liability/executor`` - gets rosbag file from IPFS and plays it
* ``/liability/infochan/channel`` - is responsible for offer, demand and result messages. It catches messages from the channel and sends signed messages back
* ``/liability/infochan/signer`` - offers services for signing offer, demand and result messages
.. * ``/liability/listener`` - watches for new liability contracts. When the event is received the node calls executor node
.. * ``/lighthouse/lighthouse`` - responsible for creating new liability contract and finalizing it
   * ``/lighthouse/matcher`` - keeps track of all incoming offers and demands. If there's a match, calls lighthouse to create a liability
   * ``/lighthouse/xrt/erc20_token`` - offers several services to work with ERC-20 tokens

And here's a list of robonomics stack topics.

.. code-block:: bash

    $ rostopic list
    /liability/complete
    /liability/current
    /liability/incoming
    /liability/infochan/incoming/ask
    /liability/infochan/incoming/bid
    /liability/infochan/incoming/result
    /liability/infochan/sending/ask
    /liability/infochan/sending/bid
    /liability/infochan/sending/result
    /liability/infochan/signing/ask
    /liability/infochan/signing/bid
    /liability/infochan/signing/result
    /liability/result
    /rosout
    /rosout_agg

The most important topics for us are:

* ``/liability/incoming`` - when a new liability is created, this topic publishes Ethereum address of the contract
* ``/liability/result`` - this topic is for publishing results. But don't publish a result directly to this topic! Use a service instead
* ``/liability/infochan/incoming/*`` - a CPS gets information about offer, demand or result from corresponding topics
* ``/liability/infochan/signing/*`` - a CPS sends offer, demand or result messages to corresponding topics

Let's start with greetings - say hello to AIRA!

You should just launch a preinstalled package ``hello_aira``::

    $ rosrun hello_aira hello_aira

We've launched our agent. It will wait for a demand message. Now it's time to send the message. Go to `dapp <https://airalab.github.io/robonomics_tutorials/>`_ and press Order. 
Now go back to the console and see the result!
