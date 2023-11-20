# rov_flir

<!-- TODO -->

## Overview

This is a boilerplate package which launches the joy_node from the joy package.

## Usage

Run the main node with

```bash
ros2 launch ps5-controller controller_launch.py
```

## Launch files

* **controller_launch.py:** Launches the joy_node from the joy package to read PS5 controller input.

## Published Topics

* **`/joy`** ([sensor_msgs/Joy])

    Outdated ROV movement instructions 

## Subscribed Topics

* **`/joy/set_feedback`** ([sensor_msgs/JoyFeedback])

[sensor_msgs/Joy]: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Joy.html
[sensor_msgs/JoyFeedback]: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JoyFeedback.html