#!/bin/bash
#
# AIRA image installation script
#

if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root" 1>&2
    exit 1
fi

echo "AIRA :: Install Docker..."
curl -sSL https://get.docker.com | sh

echo "AIRA :: Install Docker Compose..."
curl -L --fail https://github.com/docker/compose/releases/download/1.12.0/run.sh > /usr/local/bin/docker-compose
chmod +x /usr/local/bin/docker-compose

echo "AIRA :: Building image..."
git clone --recursive https://github.com/airalab/aira
cd aira-IoT/docker/share-state-liability 
docker-compose build

echo "AIRA :: Running..."
docker-compose up -d
