Basic Usage
===========

To get familiar with AIRA, let's see what is under hood. 

When you launch the client several ros nodes are started. Here's a list of robonomics communication stack nodes:
```bash
$ rosnode list
/liability/executor
/liability/infochan/channel
/liability/infochan/signer
/liability/listener
/lighthouse/infochan/channel
/lighthouse/infochan/signer
/lighthouse/lighthouse
/lighthouse/matcher
/lighthouse/xrt/erc20_token
/rosout
```

There are two the same node under different namespaces: `/liablity/infochan` and `/lighthouse/infochan`. We will describe the first one, the other one works in the same way.

* `/liability/executor` - gets rosbag file from IPFS and plays it
* `/liability/infochan/channel` - responsible for offer, demand and result messages. It retrieves messages from the channel and sends signed messages to the one
* `/liability/infochan/signer` - offers services for signing our offer, demand and result messages
* `/liability/listener` - watches for new liability contracts. When the event is received the node calls executor node
* `/lighthouse/lighthouse` - responsible for creating new liability contract and finalizing it
* `/lighthouse/matcher` - keeps track of all incoming offers and demands. If there's a match, calls lighthouse to create a liability
* `/lighthouse/xrt/erc20_token` - offers several services to work with ERC-20 tokens

And here's a list of robonomics stack topics. We skipped repetitive topics in `/liability/infochan` part.

```bash
$ rostopic list
/liability/complete
/liability/current
/liability/incoming
/liability/infochan/incoming/ask
...
/liability/infochan/signing/result
/liability/result
/lighthouse/deal
/lighthouse/infochan/incoming/ask
/lighthouse/infochan/incoming/bid
/lighthouse/infochan/incoming/result
/lighthouse/infochan/sending/ask
/lighthouse/infochan/sending/bid
/lighthouse/infochan/sending/result
/lighthouse/infochan/signing/ask
/lighthouse/infochan/signing/bid
/lighthouse/infochan/signing/result
/lighthouse/xrt/event/approval
/lighthouse/xrt/event/transfer
/rosout
/rosout_agg
```

The most important topics for us are:
* `/liability/incoming` - when a new liability is created, this topic publishes Ethereum address of the contract
* `/liability/result` - when a cyber-physical system finishes its job it has to publish a log file to IPFS. But don't publish a result directly to this topic! Use a service instead
* `/lighthouse/infochan/incoming/*` - when a CPS needs information about offer, demand or result, it subscribes on corresponding topic
* `/lighthouse/infochan/signing/*` - when a CPS needs to send offer, demand or result, it publishes to corresponding topic

What kind of message do we send to or get from `infochan`? Below is the description of a demand message:

Field           | Type      | Description                                           | Example
--------------- | --------- | ----------------------------------------------------- | ------
model           | string    | Identifier of the CPS behavioral model                | QmfXHZ2YkNC5vRjp1oAaRoDHD8H3zZznfhBPasTu348eWC
objective       | string    | Parameters of the CPS behavioral model in rosbag file | QmUo3vvSXZPQaQWjb3cH3qQo1hc8vAUqNnqbdVABbSLb6r
token           | string    | Address of operational token                          | 0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE
cost            | uint32    | Cost of the CPS behavioral model implementation       | 1
validator       | string    | Observing network address                             | 0x0000000000000000000000000000000000000000
validatorFee    | uint32    | Observing network commission                          | 0
deadline        | uint32    | Deadline block number                                 | 6393332

To send a demand message let's publish one to /lighthouse/infochan/signing/ask topic:

```bash
$ rostopic pub /lighthouse/infochan/signing/ask robonomics_lighthouse/Ask "model: 'QmfXHZ2YkNC5vRjp1oAaRoDHD8H3zZznfhBPasTu348eWC' \
objective: 'QmUo3vvSXZPQaQWjb3cH3qQo1hc8vAUqNnqbdVABbSLb6r' \
token: '0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE' \
cost: 1 \
validator: '0x0000000000000000000000000000000000000000' \
validatorFee: 0 \
deadline: 6393332"
```

An offer message should contain model, objective, token, cost, lighthouseFee and deadline fields. lighthouseFee is 0 for now.

```bash
$ rostopic pub /lighthouse/infochan/signing/bid robonomics_lighthouse/Bid "model: 'QmfXHZ2YkNC5vRjp1oAaRoDHD8H3zZznfhBPasTu348eWC'
objective: 'QmUo3vvSXZPQaQWjb3cH3qQo1hc8vAUqNnqbdVABbSLb6r'
token: '0x3cBAF1d511Adf5098511B5c5B39e1F1b506C1AFE'
cost: 1
lighthouseFee: 0
deadline: 6393332 "
```

The matcher node will do the job. When it sees the same model, objective, token and cost in demand and offer messages it creates a liability contract.

Let's pretend our cyber-physical system has done some work and now it has to finish a liability. There is a service for this purpose:
```bash
$ rosservice call /liability/finish
```
