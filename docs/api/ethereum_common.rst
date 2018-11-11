Ethereum Common
===============

The packages contains two launch files: ``erc20.launch`` and ``signer.launch``. The last one is included in `Robonomics Liability <robonomics_liability>`_.

Below is the description for ``erc20`` node which contains utils for convenient work with Ethereum accounts and XRT token.

ROS Parameters
--------------

.. py:attribute::  ~web3_http_provider

    Web3 HTTP provider address. The type is ``string``, defaults to ``http://127.0.0.1:8545``

.. py:attribute:: ~erc20_token

    ERC20 token to work with. Type is ``string``, defaults to ``xrt.3.robonomics.eth``

.. py:attribute:: ~factory_contract

    The name of the liability factory. The type is ``string``, defaults to ``factory.3.robonomics.eth``

.. py:attribute:: ~ens_contract

    The checksumed address of ENS registry. The type is ``string``, defaults to ``""``

.. py:attribute:: ~keyfile

    Path to keyfile. The type is ``string``, defaults to ``""``. **Required parameter**

.. py:attribute:: ~keyfile_password_file

    Path to a file with password for the keyfile. The type is ``string``, defaults to ``""``. **Required parameter**

Published topics
----------------

.. py:method:: /eth/event/transfer (ethereum_common/TransferEvent)

    The event `ethereum_common/TransferEvent`_ is emitted after the transfer of tokens was made

.. py:method:: /eth/event/approval (ethereum_common/ApprovalEvent)

    The event `ethereum_common/ApprovalEvent`_ is emitted after the approval of tokens was made

.. _ethereum_common/TransferEvent: ethereum_common_msg.rst
.. _ethereum_common/ApprovalEvent: ethereum_common_msg.rst

Services
--------

.. py:method:: /eth/accounts (ethereum_common/Accounts)

.. py:method:: /eth/account_eth_balance (ethereum_common/AccountBalance)

.. py:method:: /eth/eth_balance (ethereum_common/Balance)

.. py:method:: /eth/current_block (ethereum_common/BlockNumber)

.. py:method:: /eth/transfer (ethereum_common/Transfer)

.. py:method:: /eth/transfer_from (ethereum_common/TransferFrom)

.. py:method:: /eth/approve (ethereum_common/Approve)

.. py:method:: /eth/account_xrt_balance (ethereum_common/AccountBalance)

.. py:method:: /eth/xrt_balance (ethereum_common/Balance)

.. py:method:: /eth/account_xrt_allowance (ethereum_common/AccountToAddressAllowance)

.. py:method:: /eth/xrt_allowance (ethereum_common/Allowance)
