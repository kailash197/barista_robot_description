#! /usr/bin/bash
cd /home/$(whoami)/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
sudo apt update && rosdep install --from-paths src --ignore-src -y
clear && colcon build && source install/setup.bash

exec "$@"
