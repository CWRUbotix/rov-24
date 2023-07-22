# Add GZ_SIM_SYSTEM_PLUGIN_PATH to .bashrc only if it isn't already there
ROS_LINE='export GZ_SIM_RESOURCE_PATH=~/rov-24/src/surface/rov_gazebo/models:~/rov-24/src/surface/rov_gazebo/worlds'
if ! grep -qF "$ROS_LINE" ~/.bashrc ; 
    then echo "$ROS_LINE" >> ~/.bashrc ;
fi

source ~/.bashrc