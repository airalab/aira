Connecting via SSH
==================

It is more convenient to work with virtual machine via ssh connection. In this section we will configure VM.

.. attention::

    It's required to have your ssh public key on Github.com
    In case you don't have one, please follow the `link <https://help.github.com/articles/adding-a-new-ssh-key-to-your-github-account/>`_

First, launch AIRA client and run a command replacing <username> with your own::

    $ mkdir .ssh
    $ chmod 700 .ssh
    $ curl -sSL https://github.com/<username>.keys >> .ssh/authorized_keys

Now go to machine settings, network, open Advanced and then Port Forwarding

.. image:: img/4.png
   :alt: Port forwarding
   :align: center

Add a new rule:

+-----------+-----------+-----------+------------+
| Host IP   | Host Port | Guest IP  | Guest Port |
+===========+===========+===========+============+
| 127.0.1.1 | 2202      | 10.0.2.15 | 22         |
+-----------+-----------+-----------+------------+

Reboot the machine and you are able to connect to AIRA client via ssh::

    $ ssh -p 2202 root@127.0.1.1
