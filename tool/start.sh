#!/bin/bash
#
# AIRA image installation script
#

echo "AIRA :: Install Docker..."
curl -sSL https://get.docker.com | sudo sh

echo "AIRA :: Append user to docker group..."
sudo gpasswd -a `whoami` docker

echo "AIRA :: Install Docker Compose..."
curl -L https://bootstrap.pypa.io/get-pip.py | sudo python
sudo pip install docker-compose

echo "AIRA :: Building image..."
git clone --recursive https://github.com/airalab/aira
cd aira/aira-IoT/docker/share-state-liability
docker-compose build

echo "AIRA :: Running..."
docker-compose up -d

echo "AIRA :: DONE"
