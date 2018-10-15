
Overview
========

.. image:: img/3.png
   :alt: AIRA logo
   :align: center
   :scale: 50 %

AIRA (Autonomous Intelligent Robot Agent) project implements the standard of economic interaction between human-robot and robot-robot via liability smart contract. AIRA makes it possible to connect a variety of different robots to the market of robot liabilities existing on Ethereum for the direct sale of data from robot sensors, ordering of logistics services, and organization ordering of personalized products at fully automated enterprises.

Useful links 
------------

* `AIRA's official site <https://aira.life/>`_
* `The Team <https://aira.life/team>`_
* `Robonomics Network <https://robonomics.network/en/>`_

Quick Start
-----------

The first thing to do is to get the last image of AIRA. You can find it `here <https://github.com/airalab/aira/releases>`_.

.. image:: img/1.png
   :alt: Get AIRA

AIRA is distributed as virtual machine image. To launch the client you need to import .ova file to VirtualBox. You can use a convenient ``Ctrl+I`` shortcut.

It's recommended to set:

* RAM to 2Gb at least
* Network to Bridge
* At least 40 Gb SSD

When the image is imported, launch the machine. Wait until Ethereum node is fully synchronized and then AIRA client is ready to work!
AIRA creates a new address for you, it's located here::

    /var/lib/parity/foundation-env.sh

If you need to export an account, .json file is in ``/var/lib/parity/.local/share/io.parity.ethereum/keys/`` directory.

There are some helpful commands on `FAQ <faq.html>`_ page.

To become familiar with AIRA distributive we can, for example we can listen to ``/liability/infochan/incoming/ask``::

    rostopic echo /liability/infochan/incoming/ask

.. image:: img/2.png
   :alt: Check messages
