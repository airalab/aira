![AIRA](https://github.com/airalab/aira.life/raw/1a41e20ca0eed78ba9eb5376285bbe8fe961689e/wordpress/aira-2.2/assets/i/aira-logo-x2.jpg)

[![Build Status](https://travis-ci.org/airalab/aira.svg?branch=master)](https://travis-ci.org/airalab/aira)
[![GitHub release](https://img.shields.io/github/release/airalab/aira/all.svg)](https://github.com/airalab/aira/releases)

> Autonomous intelligent robot agent (AIRA) project which implements the standard of economic interaction between human-robot and robot-robot. Aira makes it possible to connect a variety of different robots to the market of robot's liabilities which existing in Ethereum.

### Airalab [Nix channel](https://nixos.org/nix/manual/#sec-channels)

- https://hydra.aira.life/project/aira/channel/latest
- Binary cache `https://hydra.aira.life`
- Public key `hydra.aira.life-1:StgkxSYBh18tccd4KUVmxHQZEUF7ad8m10Iw4jNt5ak=`

## AIRA lighthouse installation

AIRA project provide a NixOS based GNU/Linux distro which contains all set of Airalab and third-party software.

### VirtualBox images 

| Type | Arch   | SHA256 | Link |
|------|--------|--------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------|
| VirtualBox | x86_64 | 0e7c4df557097e22dda47754a5edd4fcc305b09f4c9c669efef7cb4fa2873e1e | [Download](https://github.com/airalab/aira/releases/download/0.13/aira-lighthouse-0.13-x86_64.ova)

### NixOS installation

1. git clone --recursive https://github.com/airalab/aira.git
2. add `services.lighthouse.enable = true;` to `/etc/nixos/configuration.nix`
3. nixos-rebuild switch -I nixpkgs=$(realpath aira/airapkgs)

## AIRA development cheatsheet

### Status of the system

#### Journals

```
journalctl -u ipfs -f
journalctl -u parity -f
journalctl -u lighthouse -f
```

#### IPFS peers

```
ipfs pubsub peers airalab.lighthouse.0.robonomics.eth
```

#### Lighthouse logs

```
tail -f /var/lib/lighthouse/.ros/log/latest/lighthouse-lighthouse-6.log
```

### Lighthouse development

1. Stop the service

```
systemctl stop lighthouse
```

2. Run in ROS workspace

```
cd aira/airapkgs
nix-build -A robonomics_dev
source result/setup.bash
```

```
mkdir ~/ws/src -p && cd ~/ws/src && catkin_init_workspace
git clone https://github.com/airalab/robonomics_comm
cd ..
nix-shell -p gcc
catkin_make
exit
setup devel/setup.bash
roslaunch robonomics_lighthouse lighthouse.launch
```

#### Have fun and good luck!
