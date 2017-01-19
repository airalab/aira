#!/bin/bash

CONFIG=aira_config.yaml
echo "web3uri: ${WEB3_URI}" > ${CONFIG}
echo "database: ${DATABASE}" >> ${CONFIG}
echo "telegramToken: bot${TELEGRAM_TOKEN}" >> ${CONFIG}

sleep 10
exec AiraEthBot
