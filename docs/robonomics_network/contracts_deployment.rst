Contracts Deployment
====================

Robonomics network works on top of the existing Ethereum network. The protocol is implemented by smart contracts. A source code is on `Github <https://github.com/airalab/robonomics_contracts>`_. Airalab team deploys new version of contracts and supports a current one. 

In this lesson we are going to learn more about these contracts. To do this we will deploy our test copy. Also we are going to use these contracts in the future lessons. 

You need a client running Ethereum node. You can use either one of existing network (e.g. Mainnet, Ropsten, Kovan) or your local one. For testing purpose we suggest to use this `docker container <https://github.com/f-o-a-m/cliquebait>`_ 

.. code-block:: bash

    $ docker run --rm -d -p 9545:8545 -p 9546:8546 foamspace/cliquebait:latest

Next step is obtain a copy of robonomics contracts source code::

    $ git clone --recursive https://github.com/airalab/robonomics_contracts

A file truffle.js contains available networks for migration. We will work with development network. When you are in ``robonomics_contracts`` directory install dependencies and run a migration::

    npm install // to install dependencies
    truffle migrate --network development

It's time to learn how to create a new lighthouse. For more information about Robonomics network and Lighthouse in particular read `white paper <https://robonomics.network/robonomics_white_paper_en.pdf>`_. Briefly lighthouse o distributes the running time of providers. Every lighthouse serves its own broadcast channel. Ask and Bid messages come into this channel. XRT tokens are used as a payment. 

When XRT contracts was deployed some tokens were issued on our account. Let's check the balance::

    $ truffle --network development console
    > xrt = XRT.at(XRT.address)
    > xrt.balanceOf(web3.eth.accounts[0])

And that's how we create a lighthouse::

    > factory = LiabilityFactory.at(LiabilityFactory.address)
    > tx = factory.createLighthouse(1000, 10, "test")
    > tx.then(x => {laddress = x.logs[0].args.lighthouse})
    > l = LighthouseLib.at(laddress)

Instead of deploying a lighthouse contract every time we need a new one, we ask a factory to do this job. A ``l`` variable contains lighthouse instance. The lighthouse should be able to spend our tokens. Let's make an approve and check everything went well::

    > xrt.approve(l.address,1000)
    > xrt.allowance(web3.eth.accounts[0],l.address)

And a very important step is become a worker::

    > l.refill(1000)

Each worker has to put a stake. In this case it's 1000 Wn.

Below is a table of our addresses:

+------------------+--------------------------------------------+----------------------------------+
| Contract         | Address                                    | ENS name                         |
+==================+============================================+==================================+
| ENSRegistry      | 0x80c77a7de64a15450bb8cf45ece4fbb7bae6fb49 |                                  |
+------------------+--------------------------------------------+----------------------------------+
| XRT              | 0x673583a369eb3a830a5571208cf6eb7ce83987f8 | xrt.3.robonomics.eth             |
+------------------+--------------------------------------------+----------------------------------+
| LiabilityFactory | 0x1b3190e00c1903266862af1f31714d4b81ef59b2 | factory.3.robonomics.eth         |
+------------------+--------------------------------------------+----------------------------------+
| Lighthouse       | 0xd2b78c032b6c8851a8b6cbf950caa02a77618d8e | test.lighthouse.3.robonomics.eth |
+------------------+--------------------------------------------+----------------------------------+
