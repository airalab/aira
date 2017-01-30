FROM debian:jessie
MAINTAINER Alexander Krupenkin <mail@akru.me>
LABEL Description="Aira Ethereum bot" Vendor="Airalab" Version="0.3"

RUN apt-get update \
    && apt-get install -y wget git pkg-config \
        libpcre3-dev libmysqlclient-dev libpq-dev libsqlite3-dev
RUN wget -qO- https://get.haskellstack.org/ | sh
RUN apt-get autoclean

RUN git clone --recursive https://github.com/airalab/aira -b ${GIT_BRANCH:-master}
RUN cd /aira/AiraEthBot && stack setup
RUN cd /aira/AiraEthBot && stack install
RUN mv /root/.local/bin/AiraEthBot /usr/local/bin

RUN rm -rf /aira /root/.stack /root/.local

ADD docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
ENTRYPOINT ["docker-entrypoint.sh"]
