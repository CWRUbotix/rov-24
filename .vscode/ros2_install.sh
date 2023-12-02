#!/bin/bash

# Upgrade first
sudo apt-get update
sudo apt-get upgrade -y

# Ubuntu Universe install
sudo apt-get install software-properties-common -y
sudo add-apt-repository universe -y

# Adding ROS 2 repo to system
sudo apt-get update
sudo apt-get install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update
sudo apt-get upgrade -y

# Install ROS2
sudo apt-get install ros-humble-desktop -y
sudo apt-get upgrade -y

# Add setup.bash to .bashrc only if it isn't already there
ROS_LINE='source /opt/ros/humble/setup.bash'
if ! grep -qF "$ROS_LINE" ~/.bashrc ; 
    then echo "$ROS_LINE" >> ~/.bashrc ;
fi

# Done to suppress setup.py install deprecated warnings
# Can be removed once ROS redoes their python build system
PYTHON_WARNINGS_LINE='PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources; export PYTHONWARNINGS'
if ! grep -qF "$PYTHON_WARNINGS_LINE" ~/.bashrc ; 
    then echo "$PYTHON_WARNINGS_LINE" >> ~/.bashrc ;
fi

source ~/.bashrc

# Start rosdep
rosdep update
source ~/.bashrc
