# surface_main

## Overview

This package is for launching all of our surface code. It also includes the install script for ROS.

## Installation

To install ROS

```bash
source ROS2-Install-Humble.sh
```

## Usage

Run the main node with

```bash
ros2 launch surface_main surface_all_nodes_launch.py
```

## Launch files

* **surface_all_nodes_launch.py:** Launches all surface nodes.

* **surface_operator_launch.py:** Launches operator gui and all related nodes.

* **surface_pilot_launch.py:** Launches pilot gui and all related nodes.
