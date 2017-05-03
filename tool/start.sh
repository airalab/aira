#!/bin/bash
#
# AIRA image installation script
#

echo "AIRA :: Install Docker..."
curl -sSL https://get.docker.com | sudo sh

echo "AIRA :: Install Docker Compose..."
curl -L --fail https://github.com/docker/compose/releases/download/1.12.0/run.sh > sudo tee /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

echo "AIRA :: Building image..."
git clone --recursive https://github.com/airalab/aira
cd aira/aira-IoT/docker/share-state-liability
docker-compose build

echo "AIRA :: Running..."
docker-compose up -d
