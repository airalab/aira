Introduction
============

`Robonomics-js <https://github.com/airalab/robonomics-js>`_ is a simple Javascript library for working with Robonomics network

Installation
------------

.. code-block:: bash

    npm install robonomics-js --save

or

.. code-block:: bash

    yarn add robonomics-js

CDN

.. code-block:: html

    <script src="https://cdn.jsdelivr.net/npm/robonomics-js/dist/robonomics.min.js"></script>

Dependencies 
~~~~~~~~~~~~~

* `Web3 <https://github.com/ethereum/web3.js/>`_
* `Ipfs <https://github.com/ipfs/js-ipfs>`_


Initialization
--------------

.. code-block:: javascript

    import Robonomics, { MessageProviderIpfsApi } from 'robonomics-js'
    import IPFS from 'ipfs-api'
    ​
    const robonomics = new Robonomics({
        provider: new MessageProviderIpfsApi(new IPFS('http://localhost:5001'))
    })
    ​
    robonomics.ready().then(() => {
        console.log('robonomics js ready')
        console.log('xrt', robonomics.xrt.address)
        console.log('factory', robonomics.factory.address)
        console.log('lighthouse default', robonomics.lighthouse.address)
    })

Available arguments
~~~~~~~~~~~~~~~~~~~~

* ``web3`` - isn't necessary if `Metamask <http://metamask.io/>`_ is available
* ``account`` - isn't necessary if `Metamask <http://metamask.io/>`_ is available
* ``privateKey`` - optional
* ``provider`` - IPFS HTTP API 
* ``version`` - the latest by default
* ``ens`` - ENS address, `0x314159265dD8dbb310642f98f50C066173C1259b <https://etherscan.io/address/0x314159265dD8dbb310642f98f50C066173C1259b>`_ by default
* ``lighthouse`` - a lighthouse name in ENS, airalab.lighthouse.1.robonomics.eth by default


