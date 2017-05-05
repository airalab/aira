## Raspberry Pi installation

**!!! WARNING !!! This is alpha software. It is not yet production ready, so you should use at your own risk.** 

*In tests I use RPi3 with armhf Rapbian distro.*

### Preparing

1. Simple download Raspbian [image](https://www.raspberrypi.org/downloads/raspbian/).
2. Dump image to flash card: `dd if=raspbian-jessie-lite.img of=/dev/sdb bs=1M`.
3. Insert MicroSD into RPi, connect monitor/keyboard/Ethernet.
4. Power on!

### Installation

Lets connect to Raspberry over HDMI/Keyboard or SSH.
Login `pi`, password `raspberry`.

#### Build Parity image for ARM

1. git clone https://github.com/airalab/parity-cloud -b armhf
2. cd parity-cloud && docker build .
3. docker tag %%BUILD_HASH%% airalab/parity-cloud:armhf

#### Build IPFS image for ARM

1. git clone https://github.com/airalab/ipfs-cloud -b armhf
2. cd ipfs-cloud && docker build .
3. docker tag %%BUILD_HASH%% airalab/ipfs-cloud:armhf

#### Build AIRA images for ARM

1. curl -s http://ipfs.io/ipfs/QmWhmBBHvmVdR7bf3yb8sc94caNxWHKxi7bR4fNTXGUARP | bash

*This should fail in building step because it images x86-64 only, fix it:*

2. cd aira/aira-IoT/docker/share-state-liability
3. sed -i 's/node/armhf\/node/' ../market/Dockerfile
4. sed -i 's/ubuntu/armhf\/ubuntu/' ../agent/Dockerfile
5. sed -i 's/FROM ros/FROM droneemployee\/ros-base-armhf/' Dockerfile
6. sed -i 's/ipfs-cloud/ipfs-cloud:armhf/' docker-compose.yaml
7. sed -i 's/parity-cloud:unstable/parity-cloud:armhf/' docker-compose.yaml
8. docker-compose build

#### Start

```docker-compose up -d```
