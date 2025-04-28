#!/bin/bash
# shellcheck disable=SC1090,SC1091
set -e

# setup ros2 environment
source /opt/ros/"$ROS_DISTRO"/setup.bash --
source /home/rosdev/ros2_ws/install/local_setup.bash --
sudo chmod a+rw /dev/i2c-*

# add sourcing to .bashrc
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo "source /home/rosdev/ros2_ws/install/local_setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc


exec "$@"
