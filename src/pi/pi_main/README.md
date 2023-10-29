# Pi Main

## Overview

This package launches the rest of the Pi packages. It should be run on Pi boot up.

[Tutorial followed](https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/)

## Setup

### Flashing

If something ever happens to the pi follow [this](https://www.jeffgeerling.com/blog/2020/how-flash-raspberry-pi-os-compute-module-4-emmc-usbboot) tutorial on reflashing it.

### Setup Pi SSH access over Ethernet
- Using mouse and keyboard, connect to the pi and edit `/etc/netplan/50-cloud-init.yaml`. It should look like this:
```
network:
    ethernets:
        eth0:
            dhcp4: no
            addresses: [192.168.2.1/24]
            gateway4: 192.168.2.2
            optional: true
    version: 2
```
- On windows, setup your ethernet settings by following [this tutorial](https://www.trendnet.com/press/resource-library/how-to-set-static-ip-address). You should set your static ip address to 192.168.2.1
- Connct the pi to your PC with an ethernet cable
- SSH to rov@192.168.2.1

## Installation

You need to run these commands to get the launch file running on Pi boot:

```bash
ros2 run pi_main install 
```

```bash
sudo systemctl daemon-reload
```

These commands should be run in the `src` folder after a colcon build in the workspace folder.

WARNING: Python packages must be installed with sudo for startup code to see them.

### Adding udev Rules

This should automatically be done by the prior command `ros2 run pi_main install`. If not, copy all the .rules files from `udev_rules` in this package to the `/etc/udev/rules.d` directory to use USB devices properly.

## Usage

[Tutorial followed](https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/)

### Testing without Rebooting

Installing & setting up this package creates a startup task called `cwrubotix_pi`. You can manually start and stop this task.

You should run the `cwrubotix_pi` task in the foreground for testing (**make sure to kill the background task first - see below**):

```bash
sudo cwrubotix_pi-start
```

To run the `cwrubotix_pi` task in the background (happens on Pi startup):

```bash
sudo systemctl start cwrubotix_pi.service
```

To kill the `cwrubotix_pi` background task (**do this before starting the foreground task**):

```bash
sudo systemctl stop cwrubotix_pi.service
```

To completely uninstall the `cwrubotix_pi` task:

```bash
ros2 run robot_upstart uninstall cwrubotix_pi
```

## Launch files

- **pi_launch.py**: launch the manipulators, camera streaming, and pixhawk packages
