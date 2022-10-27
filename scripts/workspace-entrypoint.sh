#!/bin/bash
# Build ROS dependency
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=69" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash
export ROS_DOMAIN_ID=69
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

sudo apt-get update
rosdep update

cd colcon_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
source ~/colcon_ws/install/setup.bash

$@
