FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"] 

# Black Magic for sshkeys
ARG ssh_prv_key
ARG ssh_pub_key
ARG email
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Authorize SSH Host
RUN mkdir -p /root/.ssh && \
    chmod 0700 /root/.ssh

# Add the keys and set permissions
RUN echo "$ssh_prv_key" > /root/.ssh/id_rsa && \
    echo "$ssh_pub_key" > /root/.ssh/id_rsa.pub && \
    chmod 600 /root/.ssh/id_rsa && \
    chmod 600 /root/.ssh/id_rsa.pub

# # Sets default email for commiting
RUN git config --global user.email "$email"

RUN source /opt/ros/humble/setup.sh \
    && rosdep update

RUN sudo apt update -y
RUN sudo apt install python3-pip -y

WORKDIR /rov-24

COPY . .

RUN git pull

# Installs ROS dependencies
RUN source /opt/ros/humble/setup.sh \
    && rosdep install --from-paths src --ignore-src -r -y

# Crazy one liner for install python dependencies
RUN for d in src/pi/*/ src/surface/*/; do pip install -e "$d"; done

RUN source /opt/ros/humble/setup.sh \
    && PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources; export PYTHONWARNINGS\
    && colcon build --symlink-install

# ENTRYPOINT ["ls"]

# sudo docker build . -t rov-24 --build-arg ssh_prv_key="$(cat ~/.ssh/id_ed25519)" --build-arg ssh_pub_key="$(cat ~/.ssh/id_ed25519.pub)" --build-arg email="$(git config user.email)"
# sudo docker run -it rov-24