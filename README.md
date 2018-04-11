![AIRA](https://github.com/airalab/aira.life/raw/master/wordpress/aira-2.2/assets/i/aira-logo-x2.jpg)

[![Build Status](https://travis-ci.org/airalab/aira.svg?branch=master)](https://travis-ci.org/airalab/aira)
[![GitHub release](https://img.shields.io/github/release/airalab/aira/all.svg)](https://github.com/airalab/aira/releases)

> Autonomous intelligent robot agent (AIRA) project which implements the standard of economic interaction between human-robot and robot-robot. Aira makes it possible to connect a variety of different robots to the market of robot's liabilities which existing in Ethereum.

### Airalab [Nix channel](https://nixos.org/nix/manual/#sec-channels)

- https://hydra.aira.life/project/aira/channel/latest
- Binary cache `https://hydra.aira.life`
- Public key `hydra.aira.life-1:StgkxSYBh18tccd4KUVmxHQZEUF7ad8m10Iw4jNt5ak=`

## AIRA installation

AIRA project provide a NixOS based GNU/Linux distro which contains all set of Airalab and third-party software.

### Hardware/VM installation

The first, fetch installation image:

| Type | Arch   | SHA256 | Link |
|------|--------|--------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------|
| VirtualBox | x86_64 | 04ba6ba45396fa1cff655624ccfc4e979f77e3e9b3235500891226ae734082dd | [Download](https://github.com/airalab/aira/releases/download/0.12/aira-lighthouse-x86_64.ova)

### Existing NixOS installation

On existing NixOS instance fortunately you can use the channels.

Append the AIRA channel:

``` bash
$ nix-channel --add https://hydra.aira.life/project/aira/channel/latest aira
$ nix-channel --update
```

So, if you dont want to compile the code on your local machine, you can add binary cache witch contains precompiled and tested AIRA packages.

Appen the AIRA binary cache into Nix configuration:

```nix
{
  nix.binaryCaches = [ https://cache.nixos.org https://hydra.aira.life ];
  nix.binaryCachePublicKeys = [ "hydra.aira.life-1:StgkxSYBh18tccd4KUVmxHQZEUF7ad8m10Iw4jNt5ak=" ];
}
```

## AIRA configuration

The next is configure your AIRA instance.

Minimal `/etc/nixos/configuration.nix` of AIRA `Game of trains` release is

```nix
{ config, ... }:

{
  boot.loader.grub.device = "/dev/sda";
  fileSystems."/".label = "nixos";

  services = {
    parity.enable = true;
    parity.chain = "kovan";

    railway-game.enable = true;
  };

  users.extraUsers.root.initialHashedPassword = "";
}

```

This starts AIRA railway Z21 controller and Parity Ethereum node on KOVAN blockchain.

Configuration is will be applyed by

```bash
$ nixos-rebuild switch
```

Logs can be in view by 

```bash
$ journalctl -u railway-market-switch -u parity -f
```

#### Have fun and good luck!
