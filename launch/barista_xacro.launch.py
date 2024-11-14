#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

'''
This file is responsible for starting Gazebo and spawning the robot model into the simulation.
Additionally, it also initiates Rviz with the robot model and laser scan data configured.
'''

def generate_launch_description():

    # Get the package directories for gazebo_ros and your custom robot description
    urdf_file = 'barista_robot_model.urdf.xacro'
    package_description = "barista_robot_description"
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    share_dir = get_package_share_directory(package_description)
    install_dir = get_package_prefix(package_description)

    # Set the path to the model files inside your package
    gazebo_models_path = os.path.join(share_dir, 'models')

    # Update environment variables for Gazebo to locate models and plugins
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] +=  ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] += ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    # Launch the Gazebo environment with a default empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    #gazebo
    position_xyz = [0.0, 0.0, 0.2]
    orientation_rpy = [0.0, 0.0, 0.0]
    robot_base_name = "barista_bot"
    entity_name = robot_base_name + "-1"
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', entity_name,
                   '-x', str(position_xyz[0]),
                   '-y', str(position_xyz[1]),
                   '-z', str(position_xyz[2]),
                   '-R', str(orientation_rpy[0]),
                   '-P', str(orientation_rpy[1]),
                   '-Y', str(orientation_rpy[2]),
                   '-topic', '/robot_description'
                   ]
    )
    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "xacro", urdf_file)

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='barista_bot_robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}]
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'urdf_vis.rviz')
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    # create and return launch description object
    return LaunchDescription(
        [   DeclareLaunchArgument( 'world',
            default_value=[os.path.join(share_dir, 'worlds', 'barista_world.world'), ''],
            description='SDF world file'),
            gazebo,
            spawn_robot,
            robot_state_publisher_node,
            rviz_node
        ]
    )
