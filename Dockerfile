FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"] 

RUN source /opt/ros/humble/setup.sh \
    && rosdep update

RUN sudo apt update -y
RUN sudo apt install python3-pip -y

WORKDIR /rov-24

COPY . .

# Installs ROS dependencies
RUN source /opt/ros/humble/setup.sh \
    && rosdep install --from-paths src --ignore-src -r -y

# Crazy one liner for install python dependencies
RUN for d in src/pi/*/ src/surface/*/; do pip install -e "$d"; done

RUN source /opt/ros/humble/setup.sh \
    && PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources; export PYTHONWARNINGS\
    && colcon build --symlink-install

COPY src/surface/surface_main/scripts/ros2_entrypoint.sh /ros_entrypoint.sh

# sudo docker build . -t rov-24 
# sudo docker run -it rov-24