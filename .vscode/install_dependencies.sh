#!/bin/bash

# Installs ROS dependencies
source /opt/ros/humble/setup.sh && rosdep install --from-paths src --ignore-src -r -y

# Crazy one liner for install python dependencies
for d in src/pi/*/ src/surface/*/; do sudo pip install -e "$d"; done
