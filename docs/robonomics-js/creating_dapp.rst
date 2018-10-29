Creating Dapp
=============

Almost every project needs a user interface to interact with. A user should not type in a `Demand` message. In Airalab repository there's a convenient template for a Dapp. In this section you are going to learn how to get a new Dapp for your CPS.

.. note:: 

    The source code is `here <https://github.com/airalab/vue-dapp-robonomics-template/>`_

To get a template you don't even have to clone the repo. Instead do these steps:

.. code:: bash

    $ npm install -g vue-cli
    $ vue init airalab/vue-dapp-robonomics-template my-project
    $ cd my-project
    $ npm install
    $ npm run dev

After the last step a webserver has started on `http://localhost:8000/ <http://localhost:8000/>`_. But before you open this link in a browser you should configure the Dapp.

.. note::

   `MetaMask <https://metamask.io>`_ is required for the Dapp

Here is a configuration file below. You have to specify a ``LIGHTHOUSE`` you work on, your CPS ``MODEL`` and ``OBJECTIVE``. Also the Dapp uses IPFS message broker. You can either set up your own `broker <https://github.com/vol4tim/ipfs-api-pubsub-ws>`_ or use existing one, for example `https://wss.pool.aira.life`.

.. code:: js

    export const NETWORK = 1
    export const LIGHTHOUSE = 'airalab.lighthouse.3.robonomics.eth'
    export const MODEL = 'QmdFh1HPVe7H4LrDio899mxA7NindgxqiNUM9BNnBD7ryS'
    export const OBJECTIVE = 'QmbSW1E73DKUvGDrgx8GirEVfHJLvj8RBijtH9iEZ7UecU'
    export const IPFS_PUBSUB = 'http://127.0.0.1:9999'
    export const ENS = ''
    export const VERSION = 1

After editing the file, launch the Dapp

.. code:: bash

    $ npm run dev

Check the source code out to get familiar with the structure of the template. 

Good luck!
