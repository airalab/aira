#!/bin/bash

CONFIG=aira_config.yaml
echo "timeout: 10" > ${CONFIG} 
echo "web3uri: ${WEB3_URI}" >> ${CONFIG}
echo "token: ${AUTH_TOKEN}" >> ${CONFIG}

sleep 10
exec AiraEthBot
