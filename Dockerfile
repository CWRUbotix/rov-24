FROM osrf/ros:iron-desktop-full

RUN apt-get update -y

# Install pip
RUN apt-get install python3-pip -y

# Install Video for Linux
RUN apt-get install v4l-utils -y

# Install lsusb
RUN apt-get install usbutils -y

# Install nano
RUN apt-get install nano -y

RUN apt-get update -y
RUN apt-get upgrade -y

WORKDIR /root/rov-24

COPY . .

# TODO for future nerd to do this via ENTRYPOINT which be better but, I could not get ENTRYPOINT to play with VsCODE.
RUN . /root/rov-24/.vscode/rov_setup.sh
RUN echo "export PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources" >> ~/.bashrc ;

# Installs ROS and python dependencies
RUN . /root/rov-24/.vscode/install_dependencies.sh

RUN . /opt/ros/iron/setup.sh \
    && PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources; export PYTHONWARNINGS\
    && colcon build --symlink-install


# https://github.com/hadolint/hadolint/wiki/DL4006
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
# Remove identity file generated by action/checkout
# Start by finding all files ending in config
# Then removes file paths without "git"
# Then removes the 'sshCommand' line from each file
RUN  find . -name "*config" | grep git | while read -r line; do sed -i "/sshCommand/d" $line; done
