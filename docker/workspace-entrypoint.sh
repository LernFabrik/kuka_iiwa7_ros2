#!/bin/bash
# Build ROS dependency
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=69" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash
export ROS_DOMAIN_ID=69

sudo apt-get update
rosdep update

$@
