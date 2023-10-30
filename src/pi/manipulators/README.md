# manipulators

<!-- TODO rewrite -->

## Overview

This package is used to toggle manipulators plugged into the I2C bus on the motherboard.

## Installation

These should be done for you but, in case something has gone wrong use these command and it should fix your installation.

```bash
git submodule add git@github.com:leloup314/TCA9555.git src/pi/manipulators/TCA9555
```

```bash
pip install wiringpi
```

## Usage

Run the main node with

```bash
ros2 run manipulators manipulator
```

Run the test node with

```bash
ros2 run manipulators test
```

## Launch files

* **manip_launch.py:** Launches the manipulators node.

## Nodes

### manipulator

On receiving a msg it toggles a manipulator on or off.

#### Subscribed Topics

* **`/manipulator_control`** ([msg/Manip])

    The control msg for activating manipulators.

### test

Sends on and off signals to the IC2 board to confirm it is working. There is no ROS features being used.

[msg/Manip]:../../rov_msgs/msg/Manip.msg
