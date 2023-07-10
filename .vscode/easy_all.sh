# Done to suppress setup.py warnings
PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources; export PYTHONWARNINGS

rosdep update --rosdistro=$ROS_DISTRO
rosdep install --from-paths src --ignore-src -r -y
# Stolen from colcon build command in VsCode
colcon build --symlink-install
source install/setup.bash