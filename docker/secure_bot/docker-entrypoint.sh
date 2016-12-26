#!/bin/bash

CONFIG=config.yaml
echo "timeout: 10" > $CONFIG
echo "token: ${AUTH_TOKEN}" >> $CONFIG

sleep 10
exec AiraSecureBot
