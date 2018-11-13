![AIRA](https://github.com/airalab/aira.life/raw/1a41e20ca0eed78ba9eb5376285bbe8fe961689e/wordpress/aira-2.2/assets/i/aira-logo-x2.jpg)

[![Documentation Status](https://readthedocs.org/projects/aira/badge/?version=latest)](https://aira.readthedocs.io/en/latest/?badge=latest)
[![Build Status](https://travis-ci.org/airalab/aira.svg?branch=master)](https://travis-ci.org/airalab/aira)
[![GitHub release](https://img.shields.io/github/release/airalab/aira/all.svg)](https://github.com/airalab/aira/releases)

> Autonomous intelligent robot agent (AIRA) project which implements the standard of economic interaction between human-robot and robot-robot. Aira makes it possible to connect a variety of different robots to the market of robot's liabilities which existing in Ethereum.

Airalab [channel](https://hydra.aira.life/project/aira/channel/latest) 
=============================================================

> [Nix channel](https://nixos.org/nix/manual/#sec-channels) of Airalab community is placed on [airapkgs](https://github.com/airalab/airapkgs). Currently supported `nixos-unstable` branch only.

- Binary cache `https://hydra.aira.life`
- Public key `hydra.aira.life-1:StgkxSYBh18tccd4KUVmxHQZEUF7ad8m10Iw4jNt5ak=`

### Setup

The first, add channel to nix channel registry.

```bash
$ nix-channel --add https://hydra.aira.life/project/aira/channel/latest aira
```

To speed up package install - add airalab nix package cache to `/etc/nixos/configuration.nix`.

```
{
  nix.binaryCaches = [
    https://cache.nixos.org
    https://hydra.aira.life
  ];

  nix.binaryCachePublicKeys = [
    "hydra.aira.life-1:StgkxSYBh18tccd4KUVmxHQZEUF7ad8m10Iw4jNt5ak="
  ];
}
```

Quick Start Images
------------------

AIRA distribution is builded on stable snapshot of `airapkgs`.

### Precompiled images

| Type       | Arch   | SHA256                                                             | Link |
|------------|--------|--------------------------------------------------------------------|-----------------------------------------------------------------------------------------|
| VirtualBox | x86_64 | `d6149e47d62446a517a18b7b00661a2ae2a6865fc448ac0fee9400204940b804` | [Download](https://github.com/airalab/aira/releases/download/0.15/aira-0.15-x86_64.ova)

### Building from source

```bash
$ curl https://nixos.org/nix/install | sh
$ git clone --recursive https://github.com/airalab/aira && cd aira
$ make
```
