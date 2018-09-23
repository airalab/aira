How to
======

How to create a demand?
-----------------------

Listen to a demand with a specific model: 

.. code-block:: javascript

    const model = 'QmWXk8D1Fh5XFJvBodcWbwgyw9htjc6FJg8qi1YYEoPnrg'
    robonomics.getAsk(model, (msg) => {
        console.log(msg)
    })
    const ask = {
        objective: 'QmSt69qQqGka1qwRRHbdmAWk4nCbsV1mqJwd8cWbEyhf1M',
        token: robonomics.xrt.address,
        cost: 1,
        deadline: 9999999
    }

**Fields:**

* ``objective`` - IPFS hash to a rosbag file with a task
* ``token`` - token address
* ``cost`` - cost
* ``validator`` - validator address
* ``validatorFee`` - validator fee 
* ``deadline`` - block number 

It's necessary to make an approve:

.. code-block:: javascript

    robonomics.xrt.send('approve', [robonomics.factory.address, ask.cost], { from: robonomics.account }).then((tx) => console.log(tx))

In case of other token:

.. code-block:: javascript

    import { Token } from 'robonomics-js'
    const token = new Token(robonomics.web3, '0x1231321321321321321321321')
    token.send('approve', [robonomics.factory.address, ask.cost], { from: robonomics.account })
      .then((tx) => console.log(tx))

And send a demand message:

.. code-block:: javascript

    robonomics.postAsk(market, ask)
        .then((liability) => {
            console.log('liability', liability.address)
            liability.watchResult((result) => {
                console.log('liability result', result)
            })
            return liability.getInfo()
        })
        .then((info) => {
            console.log('liability info', info)
        })

How to get an offer?
-----------------------

Obtain all the messages by a given model:

.. code-block:: javascript

    const model = 'QmWXk8D1Fh5XFJvBodcWbwgyw9htjc6FJg8qi1YYEoPnrg'
    robonomics.getBid(model, (msg) => {
        console.log(msg)
    })

**Fields:**

* ``objective`` - IPFS hash to a rosbag file with a task
* ``token`` - token address
* ``cost`` - cost
* ``lighthouseFee`` - lighthouse fee 
* ``deadline`` - block number 

How to listen to a result?
-------------------------

Obtain all the messages by a given model:

.. code-block:: javascript

    robonomics.getResult((msg) => {
        console.log(msg)
    })

.. note::

    It's not a verified result. A verified result could be obtained from a liability contract.

How to create a lighthouse?
----------------------------

.. code-block:: javascript

    const minimalFreeze = 1000 // Wn
    const timeout = 25 // blocks
    const name = 'mylighthouse' // название маяка
    robonomics.factory.send('createLighthouse', [minimalFreeze, timeout, name], { from: robonomics.account })
        .then((tx) => console.log(tx))
    ​
    robonomics.factory.watchLighthouse((lighthouse) => {
        console.log(lighthouse.name)
    })

How to become a provider?
-------------------------

.. code-block:: javascript

    const name = 'mylighthouse' // название маяка
    const stake = 1000 // Wn
    ​
    robonomics.setLighthouse(name)
    ​
    robonomics.xrt.send('approve', [robonomics.lighthouse.address, stake], { from: robonomics.account })
        .then((tx) => console.log(tx))
    ​
    robonomics.lighthouse.send('refill', [stake], { from: robonomics.account })
      .then((tx) => console.log(tx))

How to change a lighthouse?
---------------------------

.. code-block:: javascript

    robonomics.setLighthouse(name)

How to check the balance?
--------------------------

.. code-block:: javascript

    robonomics.xrt.call('balanceOf', [robonomics.account])
    .then((balance) => console.log('balance', balance))

How to check the allowance?
----------------------------

.. code-block:: javascript

    robonomics.xrt.call('allowance', [robonomics.account, robonomics.factory.address])
        .then((allowance) => console.log('allowance', allowance))
