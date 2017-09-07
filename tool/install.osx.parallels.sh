#!/bin/bash
#
# AIRA image Mac OS installation script
#

echo "AIRA :: Install Homebrew..."
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"

echo "AIRA :: Install Docker..."
brew install docker docker-compose docker-machine docker-machine-parallels

echo "AIRA :: Create AIRA docker machine..."
docker-machine create --driver=parallels --parallels-memory=2048 aira
eval $(docker-machine env aira)

echo "AIRA :: Building image..."
git clone --recursive https://github.com/airalab/aira
cd aira/aira-IoT/docker/share-state-liability
docker-compose build

echo "AIRA :: Running..."
docker-compose up -d

echo "AIRA :: DONE"
echo "AIRA has been successfully installed, to continue open the URL in "
echo "the Chrome browser with the metamask or parity extension installed: "
docker-machine ip aira | awk '{print "http://"$0":8000"}'
