git clone https://github.com/ArduPilot/ardupilot.git --recurse-submodules ~/ardupilot
cd ~/ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
./modules/waf/waf-light configure --board sitl \
  && modules/waf/waf-light build --target bin/ardusub


# Add ardusub to .bashrc only if it isn't already there
ROS_LINE='export PATH=$HOME/ardupilot/build/sitl/bin:$PATH'
if ! grep -qF "$ROS_LINE" ~/.bashrc ; 
    then echo "$ROS_LINE" >> ~/.bashrc ;
fi

source ~/.bashrc