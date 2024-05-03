# Pi Info

## Overview

This package launches nodes to publish information about the pi such as the current ip address, and a heartbeat.

## Usage

Run the main node with

```bash
ros2 launch pi_info pi_info_launch.py
```

## Launch files

* **pi_info_launch.py:** launches the heartbeat and ip_publisher nodes

## Nodes

### heartbeat

Publishes a heartbeat at a regular time interval.

#### Published Topics

* **`/pi_heartbeat`** ([rov_msgs/msg/Heartbeat])

    The heartbeat for the pi that is published at a regular time interval.

### ip_publisher

Publishes the current ip address of the network.

#### Published Topics

* **`/ip_address`** ([rov_msgs/msg/IPAddress])

    The IP Address as a string.

[rov_msgs/msg/IPAddress]: ../../rov_msgs/msg/IPAddress.msg
[rov_msgs/msg/Heartbeat]: ../../rov_msgs/msg/Heartbeat.msg