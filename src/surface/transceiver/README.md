# transceiver

## Overview

This package is used to control when the float is submerged and keep track of what time it was submerged.

## Usage

Run the main node with

```bash
ros2 launch transceiver serial_reader_launch.py
```

Useful command:
sudo usermod -a -G dialout $USER

## Launch files

* **serial_reader_launch.py:** launches the SerialReader Node

## Nodes

### serial_reader

Tells the float when to submerge and receives data from the float.

#### Subscribed Topics

* **`/transceiver_control`** ([msg/FloatCommand.msg])

    When to submerge the float and what time it was submerged.

#### Published Topics

* **`/transceiver_data`** ([msg/FloatCommand.msg])

    The data received from the float.


[msg/FloatCommand.msg]:../../rov_msgs/msg/FloatCommand.msg