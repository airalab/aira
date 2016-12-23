#!/bin/bash

# First init
if [ ! -f password.txt ]; then
  choose() { echo ${1:RANDOM%${#1}:1} $RANDOM; }
  echo "$({ choose '!@#$%^\&'
    choose '0123456789'
    choose 'abcdefghijklmnopqrstuvwxyz'
    choose 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
    for i in $( seq 1 $(( 4 + RANDOM % 8 )) )
    do
      choose '0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ'
    done
    } | sort -R | awk '{printf "%s",$1}')" > password.txt
  parity account new --password password.txt 2>&1 | awk '{print $4}' > address.txt 
fi

ADDRESS="$(cat address.txt)"
CHAIN="${CHAIN:-ropsten}"
echo -e "Start Parity with:\n\tAddress 0x${ADDRESS}\n\tChain: ${CHAIN}"

exec parity --warp \
            --no-ipc \
            --no-dapps \
            --jsonrpc-interface '0.0.0.0' \
            --jsonrpc-hosts='all' \
            --unlock $ADDRESS \
            --password password.txt \
            --chain=$CHAIN $@
