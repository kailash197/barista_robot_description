#! /usr/bin/bash
cd /home/$(whoami)/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
sudo apt update && rosdep install --from-paths src --ignore-src -y
clear && colcon build && source install/setup.bash

echo "Please run the following command to start the tmux session:"
echo "cd ~/ros2_ws/startup && ./start_session.sh"

exec "$@"
