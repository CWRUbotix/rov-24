# Add GZ_SIM_SYSTEM_PLUGIN_PATH to .bashrc only if it isn't already there
ROS_LINE="export IGN_GAZEBO_RESOURCE_PATH=$(pwd)/src/surface/rov_gazebo/models:$(pwd)/src/surface/rov_gazebo/models/props:$(pwd)/src/surface/rov_gazebo/worlds:$IGN_GAZEBO_RESOURCE_PATH"
if ! grep -qF "$ROS_LINE" ~/.bashrc ; 
    then echo "$ROS_LINE" >> ~/.bashrc ;
fi

source ~/.bashrc