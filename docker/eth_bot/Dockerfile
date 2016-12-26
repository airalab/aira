FROM akru/aira
MAINTAINER Alexander Krupenkin <mail@akru.me>
LABEL Description="Aira Ethereum bot" Vendor="Airalab" Version="0.1"

RUN cd /aira && git pull && git submodule update
RUN cd /aira/AiraEthBot && stack install
RUN mv /root/.local/bin/AiraEthBot /usr/local/bin

ADD docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
ENTRYPOINT ["docker-entrypoint.sh"]
