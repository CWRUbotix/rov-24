sudo apt update
sudo apt install libgz-sim7-dev rapidjson-dev
git clone https://github.com/ArduPilot/ardupilot_gazebo ~/ardupilot_gazebo
cd ~/ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4

# Add GZ_SIM_SYSTEM_PLUGIN_PATH to .bashrc only if it isn't already there
ROS_LINE='export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH'
if ! grep -qF "$ROS_LINE" ~/.bashrc ; 
    then echo "$ROS_LINE" >> ~/.bashrc ;
fi

source ~/.bashrc