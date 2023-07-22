git clone https://github.com/ArduPilot/ardupilot.git --recurse-submodules ~/ardupilot
cd ~/ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
./modules/waf/waf-light configure --board sitl \
  && modules/waf/waf-light build --target bin/ardusub

