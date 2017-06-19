## x86-64 Installation 

### One-line installer

```bash
curl -s http://ipfs.io/ipfs/QmWhmBBHvmVdR7bf3yb8sc94caNxWHKxi7bR4fNTXGUARP | bash
```

This should clone and build aira images and running up at all.

### After install

Also spawned 8000 port for admin page.

The `state` directory is shared, this means that all directory content is packed
and sended as IPFS hash by liability contract at all time. You should place your
usefull content in this directory, e.g. sensor logs, charts, images, etc.

### The Market

Liability market contract & dApp provide liability trading functionality. Use them. 

https://www.youtube.com/embed/auAK4U0HuEY?list=PLLepqB9oh7WubmFjnB-g9a5x6vjTHNR6F

https://www.youtube.com/watch?v=2ujSXUsRHJs&list=PLLepqB9oh7WubmFjnB-g9a5x6vjTHNR6F&index=3

#### How it works

0. The Operator setup AIRA by admin page:
    - Market
    - Liability bid price
    - Liability ask response price
1. Market emit close order event (for AIRA orders) or AIRA found suitable order to buy 
2. By market contract created Liability contract
3. AIRA catch liability address from market by event
4. IPFS recursive add `share` directory
5. AIRA publish IPFS hash

