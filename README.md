
            *
           /|
          / |
         /  |
        *   *       _    ___ ____      _    
       /|  /|      / \  |_ _|  _ \    / \   
      / | / |     / _ \  | || |_) |  / _ \  
     /  |/  |    / ___ \ | ||  _ <  / ___ \ 
    *---*---*   /_/   \_\___|_| \_\/_/   \_\


[![Documentation Status](https://readthedocs.org/projects/aira/badge/?version=latest)](https://aira.readthedocs.io/en/latest/?badge=latest)
[![Build Status](https://travis-ci.org/airalab/aira.svg?branch=master)](https://travis-ci.org/airalab/aira)
[![GitHub release](https://img.shields.io/github/release/airalab/aira/all.svg)](https://github.com/airalab/aira/releases)

> Autonomous intelligent robot agent (AIRA) project which implements the standard of economic interaction between human-robot and robot-robot. Aira makes it possible to connect a variety of different robots to the market of robot's liabilities which existing in Ethereum.

Airalab Nix channel
===================

> [Nix channel](https://nixos.org/nix/manual/#sec-channels) of Airalab community is placed on [airapkgs](https://github.com/airalab/airapkgs). Currently supported `nixos-unstable` branch only.

- Binary cache: `https://aira.cachix.org`
- Public key: `aira.cachix.org-1:/5nHPqhVrtvt7KCk04I8cH/jETANk8BtPHWsEtcwU/M=`

### Setup

The first, add channel to nix channel registry.

```bash
$ nix-channel --add https://aira.life/channels/aira-stable aira
```

OR using [IPFS](https://ipfs.io)

```bash
$ nix-channel --add http://localhost:8080/ipns/stable.releases.aira.life aira
```

To speed up package install - add airalab nix package cache to `/etc/nixos/configuration.nix`.

```
{
  nix.binaryCaches = [
    https://cache.nixos.org
    https://aira.cachix.org
  ];

  nix.binaryCachePublicKeys = [
    "aira.cachix.org-1:/5nHPqhVrtvt7KCk04I8cH/jETANk8BtPHWsEtcwU/M="
  ];
}
```

Quick Start Images
------------------

AIRA distribution is builded on stable snapshot of `airapkgs`.

### Precompiled images

| Type       | Arch   | SHA256                                                             | Link |
|------------|--------|--------------------------------------------------------------------|-----------------------------------------------------------------------------------------|
| VirtualBox | x86_64 | `e936c31a5e20d1e4b2aa7f3c03eb69c3fc60a4e80d3de3772badbd5f612c647e` | [Download](https://releases.aira.life/channels/aira/unstable/862-aira-unstable/nixos-20.03pre-git-x86_64-linux.ova) |
| SD Image   | AArch64 | `35f580bed14b49f115ad4ec798d7f26c69eaef05a7371f5bd7ec323452ce549b` | [Download](https://releases.aira.life/channels/aira/unstable/862-aira-unstable/nixos-sd-image-20.03pre-git-aarch64-linux.img) |

### Building from source

```bash
$ curl https://nixos.org/nix/install | sh
$ git clone --recursive https://github.com/airalab/aira && cd aira
$ make
```
