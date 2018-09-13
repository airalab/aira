Robonomics Contracts Deployment
===============================

Robonomics network works on top of the existing Ethereum network. The protocol is implemented by smart contracts. A source code is on [Github](https://github.com/airalab/robonomics_contracts). Airalab team deploys new version of contracts and supports a current one. 

In this lesson we are going to learn more about these contracts. To do this we will deploy our test copy. Also we are going to use these contracts in the future lessons. 

Here and further we will work in Kovan test network and use [Parity](https://paritytech.io/) client.

> If you have a fresh installed client, you have to create a new account:
> ```
> $ parity account new
> ```
> Also you'll need ethers for sending transactions. Choose a preferred way to get some on [this](https://github.com/kovan-testnet/faucet) page or ask Airalab team in telegram [chat](https://aira.life/chat) 

Launch the client. In arguments we say which network to work with, allow to use JSONRPC API, choose a port for API and unlock the account. A file pass contains password.
```
$ parity --chain kovan --jsonrpc-hosts all --jsonrpc-interface all --jsonrpc-cors '*' --jsonrpc-apis 'web3,eth,net,parity,traces,rpc,parity_set,personal' --jsonrpc-port 9545 --unlock <account> --password ./pass
```

While the network is syncing, let's obtain a copy of robonomics contracts source code:
```
$ git clone --recursive https://github.com/airalab/robonomics_contracts
```

A file truffle.js contains available networks for migration. We will work with development network. When you are in robonomics_contracts directory install dependencies and run a migration:
```
npm install // to install dependecies
truffle migrate --network development
```

It's time to learn how to create a new lighthouse. For more information about Robonomics network and Lighthouse in particular read [white paper](https://robonomics.network/robonomics_white_paper_en.pdf). Briefly lighthouse o distributes the running time of providers. Every lighthouse serves its own broadcast channel. Ask and Bid messages come into this channel. XRT tokens are used as a payment. 

When XRT contracts was deployed some tokens were issued on our account. Let's check the balance:
```
$ truffle --network development console
> xrt = XRT.at(XRT.address)
> xrt.balanceOf(web3.eth.accounts[0])
```

And that's how we create a lighthouse:
```
> factory = LiabilityFactory.at(LiabilityFactory.address)
> tx = factory.createLighthouse(1000, 10, "test")
> laddress = tx.then(x => x.logs[0].args.lighthouse)
> l = LighthouseLib.at(laddress)
```

Instead of deploying a lighthouse contract every time we need a new one, we ask a factory to do this job. A `l` variable contains lighthouse instance. The lighthouse should be able to spend our tokens. Let's make an approve and check everything went well:
```
> xrt.approve(l.address,1000)
> xrt.allowance(web3.eth.accounts[0],l.address)
```

And a very important step is become a worker: 
```
> l.refill(1000)
```
Each worker has to put a stake. In this case it's 1000 Wn.

Below is a table of our addresses:

Contract            | Address                                       | ENS name
------------------- | --------------------------------------------- | --------
ENSRegistry         | 0x9bCF5Ec30461f153B11a70e9f978b05B866870a9    |
XRT                 | 0x9f0851E09d42685d6536f44Bf181D2D596bAF7e6    | xrt.1.robonomics.eth
LiabilityFactory    | 0xe57c6c673aa27EE4f0C471b8D8161dB8d733A92b    | factory.1.robonomics.eth
Lighthouse          | 0xED571F3e58197B0E58AAeB935A70B58Bd2e89BAB    | test.lighthouse.1.robonomics.eth
