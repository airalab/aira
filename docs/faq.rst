Frequently Asked Questions
==========================

How to see logs from main services? 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

IPFS in real time:

.. code-block:: bash

    journalctl -u ipfs -f

and Liability::

    journalctl -u liability -f

How to check the quantity of IPFS peers?
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    ipfs pubsub peers airalab.lighthouse.0.robonomics.eth

IPFS can't connect to the daemon, what should I do? 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Try to specify ``--api`` option

.. code-block:: bash

    ipfs swarm peers --api=/ip4/127.0.0.1/tcp/5001/

