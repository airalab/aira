AIRA Developer Cheatsheet
=========================

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
source devel/setup.bash
roslaunch robonomics_lighthouse lighthouse.launch
```
