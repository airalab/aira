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

    The event :ref:`ethereum_common/TransferEvent <Ethereum-common-TransferEvent.msg>` is emitted after the transfer of tokens was made

.. py:method:: /eth/event/approval (ethereum_common/ApprovalEvent)

    The event :ref:`ethereum_common/ApprovalEvent <Ethereum-common-ApprovalEvent.msg>` is emitted after the approval of tokens was made

Services
--------

.. py:method:: /eth/accounts (ethereum_common/Accounts)

    List of available Ethereum accounts. See :ref:`Ethereum-common-Accounts.srv`

.. py:method:: /eth/account_eth_balance (ethereum_common/AccountBalance)

    Returns the balance of the given address in Wei. See :ref:`Ethereum-common-AccountBalance.srv`

.. py:method:: /eth/eth_balance (ethereum_common/Balance)

    Returns the balance of the default address. See :ref:`Ethereum-common-Balance.srv`

.. py:method:: /eth/current_block (ethereum_common/BlockNumber)

    Returns current block number. See :ref:`Ethereum-common-BlockNumber.srv`

.. py:method:: /eth/transfer (ethereum_common/Transfer)

    Transfers tokens from the default account to a given one. See :ref:`Ethereum-common-Transfer.srv`

.. py:method:: /eth/transfer_from (ethereum_common/TransferFrom)

    Transfers tokens from a given account to another one. See :ref:`Ethereum-common-TransferFrom.srv`

.. py:method:: /eth/approve (ethereum_common/Approve)

    Approves tokens from the default account to a given one. See :ref:`Ethereum-common-Approve.srv`

.. py:method:: /eth/account_xrt_balance (ethereum_common/AccountBalance)

    Returns the XRT balance of a given account. See :ref:`Ethereum-common-AccountBalance.srv`

.. py:method:: /eth/xrt_balance (ethereum_common/Balance)

    Return the XRT balance of the default account. See :ref:`Ethereum-common-Balance.srv`

.. py:method:: /eth/account_xrt_allowance (ethereum_common/AccountToAddressAllowance)

    Returns how much one account is allowed to spend from another account. See :ref:`Ethereum-common-AccountToAddressAllowance.srv`

.. py:method:: /eth/xrt_allowance (ethereum_common/Allowance)

    Returns how much the Factory is allowed to spend from the default account. See :ref:`Ethereum-common-Allowance.srv`
