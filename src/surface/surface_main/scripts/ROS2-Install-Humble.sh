#!/bin/bash

# Upgrade first
sudo apt-get update
sudo apt upgrade

# Install git, curl, pip
sudo apt install git
sudo apt install curl
sudo apt install python3-pip

# Install python packages that rosdep can't install
pip install pyserial
pip install wiringpi
pip install PyQt6
pip install pyqtdarktheme
pip install numpy
pip install numpy --upgrade  # Make sure numpy is up-do-date to avoid https://github.com/opencv/opencv-python/issues/885
pip install opencv-python
pip install opencv-python-headless

# Setting locale
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# Ubuntu Universe install
sudo apt install software-properties-common
sudo add-apt-repository universe

# Ubuntu Universe check
apt-cache policy | grep universe

# Adding ROS 2 repo to system
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade

# Missing ROS dependecies
sudo apt install python3-catkin-pkg-modules
sudo apt install python3-rosdistro-modules
sudo apt install python3-rospkg-modules
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep2

# Install humble distro of ROS2
sudo apt install ros-humble-desktop
sudo apt upgrade

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
