FROM debian:jessie 
MAINTAINER Alexander Krupenkin <mail@akru.me>
LABEL Description="Cloud Ethereum node" Vendor="Airalab" Version="0.1"
EXPOSE 30303 8545

ADD docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
ADD http://d1h4xl4cr1h0mo.cloudfront.net/v1.4.6/x86_64-unknown-linux-gnu/parity_1.4.6_amd64.deb /parity.deb
RUN dpkg -i parity.deb && rm -f parity.deb

ENTRYPOINT ["docker-entrypoint.sh"]
