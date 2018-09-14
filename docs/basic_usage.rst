Basic Usage
===========

To get familiar with AIRA, let's see what is under hood. 

When you launch the client several ros nodes are started. Here's a list of robonomics communication stack nodes::

    $ rosnode list
    /liability/executor
    /liability/infochan/channel
    /liability/infochan/signer
    /liability/listener
    /lighthouse/infochan/channel
    /lighthouse/infochan/signer
    /lighthouse/lighthouse
    /lighthouse/matcher
    /lighthouse/xrt/erc20_token
    /rosout

There are two the same node under different namespaces: ``/liablity/infochan`` and ``/lighthouse/infochan``. We will describe the first one, the other one works in the same way.

* ``/liability/executor`` - gets rosbag file from IPFS and plays it
* ``/liability/infochan/channel`` - responsible for offer, demand and result messages. It retrieves messages from the channel and sends signed messages to the one
* ``/liability/infochan/signer`` - offers services for signing our offer, demand and result messages
* ``/liability/listener`` - watches for new liability contracts. When the event is received the node calls executor node
* ``/lighthouse/lighthouse`` - responsible for creating new liability contract and finalizing it
* ``/lighthouse/matcher`` - keeps track of all incoming offers and demands. If there's a match, calls lighthouse to create a liability
* ``/lighthouse/xrt/erc20_token`` - offers several services to work with ERC-20 tokens

And here's a list of robonomics stack topics. We skipped repetitive topics in ``/liability/infochan`` part.

.. code-block:: bash

    $ rostopic list
    /liability/complete
    /liability/current
    /liability/incoming
    /liability/infochan/incoming/ask
    ...
    /liability/infochan/signing/result
    /liability/result
    /lighthouse/deal
    /lighthouse/infochan/incoming/ask
    /lighthouse/infochan/incoming/bid
    /lighthouse/infochan/incoming/result
    /lighthouse/infochan/sending/ask
    /lighthouse/infochan/sending/bid
    /lighthouse/infochan/sending/result
    /lighthouse/infochan/signing/ask
    /lighthouse/infochan/signing/bid
    /lighthouse/infochan/signing/result
    /lighthouse/xrt/event/approval
    /lighthouse/xrt/event/transfer
    /rosout
    /rosout_agg

The most important topics for us are:

* ``/liability/incoming`` - when a new liability is created, this topic publishes Ethereum address of the contract
* ``/liability/result`` - when a cyber-physical system finishes its job it has to publish a log file to IPFS. But don't publish a result directly to this topic! Use a service instead
* ``/lighthouse/infochan/incoming/*`` - when a CPS needs information about offer, demand or result, it subscribes on corresponding topic
* ``/lighthouse/infochan/signing/*`` - when a CPS needs to send offer, demand or result, it publishes to corresponding topic

Often introduction to a new technology begins with a greeting. Let's say hello to ARIA!

First of all we need to clone a Github repo with examples::

    $ git clone https://github.com/Vourhey/aira-lessons.git

The code for this section is in `hello_aira` folder. To build and launch this package do the following::

    $ mkdir -p ws/src && cd ws/src
    $ cp -r ~/aira-lessons/hello_aira . 
    $ catkin_init_workspace && cd .. && catkin_make 
    $ source devel/setup.bash
    $ rosrun hello_aira hello_aira

We've launched our agent. It will wait for a demand message. All we have to do is to send the message. Go to `Dapp <https://vourhey.github.io/aira-lessons/#/>`_ and press Order. 
Now go back to the console and see the result!
