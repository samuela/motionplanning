FROM ubuntu:14.04

RUN apt-get update && apt-get install -y curl

# RUN useradd -ms /bin/bash me && adduser me sudo
RUN adduser --system --shell /bin/bash --disabled-password me && \
    adduser me sudo
USER me
WORKDIR /home/me
ENV USER=me

RUN su me && curl https://nixos.org/nix/install | sh
