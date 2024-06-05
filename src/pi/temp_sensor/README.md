# Flood Detection

## Overview

This package launches flood detection.

## Installation

```bash
sudo apt-get install python3-smbus
```

## Usage

```bash
ros2 launch 
```

## Launch files

* **.py:**  Primary launch file for temp sensor.

## Nodes

### temp_sensor

Reads temperature.

#### Published Topics

* **`/`** ([rov_msgs/flooding])

    Whether rov is flooding.


### test

Tests flooding.

[rov_msgs/flooding]: ../../rov_msgs/msg/Flooding.msg
