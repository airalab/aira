FROM debian:jessie
MAINTAINER Alexander Krupenkin <mail@akru.me>
LABEL Description="Aira" Vendor="Airalab" Version="0.1"

RUN apt-get update && apt-get install -y wget git
RUN wget -qO- https://get.haskellstack.org/ | sh
RUN git clone --recursive https://github.com/airalab/aira -b ${GIT_BRANCH:-develop}
RUN cd aira/aira-bot-platform && stack setup
RUN sed -i "s/localhost/parity/g" aira/web3/src/Network/Ethereum/Web3/Types.hs
RUN cd aira/aira-bot-platform && stack build 
