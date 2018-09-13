Frequently Asked Questions
==========================

How to see logs from main services? 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Parity and IPFS in real time:

.. code-block:: bash

    journalctl -u ipfs -f
    journalctl -u parity -f

and Lighthouse::

    journalctl -u lighthouse -f
    # or
    tail -f /var/lib/lighthouse/.ros/log/latest/lighthouse-lighthouse-6.log

How to check the quantity of IPFS peers?
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    ipfs pubsub peers airalab.lighthouse.0.robonomics.eth


