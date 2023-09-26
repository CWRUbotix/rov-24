#!/bin/bash
set -e

# setup ros2 environment
# PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources; export PYTHONWARNINGS
# source /rov-24/install/setup.bash
source "/opt/ros/$ROS_DISTRO/setup.bash" --
exec "$@"