# Barista Robot Description

## Description
A simple robot

## Development

--To-do description--

#### Dependencies
1. Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/)

When your computer has NVIDIA GPU, consider making sure you also have the NVIDIA drivers, NVIDIA container toolkit installed:

2. NVIDIA Drivers in Ubuntu: https://documentation.ubuntu.com/server/how-to/graphics/install-nvidia-drivers/
3. NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html


## Build the docker
To build the docker images, change to the workspace directory that contains Docker files and run:
```bash
cd ~/barista_robot/barista_robot_description && sudo rm -rf build install log
cd ~/barista_robot && docker compose -f docker/docker-compose.yml build
xhost +local:docker
cd ~/barista_robot && docker compose -f docker/docker-compose.yml run --remove-orphans barista-dev
```

### Useful Commands

#### URDF
```bash
cd ~/ros2_ws && colcon build --packages-select barista_robot_description
source install/setup.bash && ros2 launch barista_robot_description barista_urdf.launch.py

ros2 topic list
ros2 topic echo /odom
ros2 topic echo /scan

```
#### XACRO
```bash
cd ~/ros2_ws && colcon build --packages-select barista_robot_description
source install/setup.bash && ros2 launch barista_robot_description barista_xacro.launch.py


ros2 launch barista_robot_description barista_two_robots.launch.py
```

#### Robot Chaser
```bash
ros2 run robot_chase robot_chase
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/morty/cmd_vel
```

#### Debug commands
```bash
ros2 topic list
ros2 run tf2_tools view_frames
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/rick/cmd_vel
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/morty/cmd_vel

ros2 run tf2_ros tf2_monitor camera_bot_base_link rgb_camera_link_frame


ros2 run <your_package_name> <your_executable> --ros-args -p robot_base_frame:="morty/base_link" -p odom_topic:="/morty/odom"

source install/setup.bash && ros2 run barista_robot_description barista_bot_odom_to_tf_pub.py --ros-args -p robot_base_frame:="morty/odom" -p odom_topic:="/morty/odom"
ros2 run tf2_ros tf2_echo rick/base_link morty/base_link
```
