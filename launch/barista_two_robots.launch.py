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
from launch.actions import TimerAction
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

    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "xacro", urdf_file)

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

    #robots
    robot_1 = {'name': 'morty',
               'color':'Blue',
               'position': [0.5, 0.5, 0.2],
               'orientation': [0.0, 0.0, 0.0] }
    robot_2 = {'name': 'rick',
               'color':'Red',
               'position': [0.0, 0.0, 0.2],
               'orientation': [0.0, 0.0, 0.0] }
    robots = [robot_1, robot_2]
    spawn_nodes = list()
    rsp_nodes = list()

    for robot in robots:
        spawn_nodes.append(
            Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-entity', robot.get('name', 'barista_bot'),
                    '-x', str(robot.get('position')[0]),
                    '-y', str(robot.get('position')[1]),
                    '-z', str(robot.get('position')[2]),
                    '-R', str(robot.get('orientation')[0]),
                    '-P', str(robot.get('orientation')[1]),
                    '-Y', str(robot.get('orientation')[2]),
                    '-topic', robot.get('name', '')+'/robot_description'
                    ]
            )
        )

        rsp_nodes.append(
            Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_node',
            namespace=robot.get('name', ''),
            emulate_tty=True,
            parameters=[{
                'use_sim_time': True,
                'frame_prefix': robot.get('name', '') + "/",
                'robot_description': Command([
                    'xacro ', robot_desc_path, ' ',
                    'include_laser:=true',
                    ' robot_name:=', robot.get('name', ''),
                    ' color:=', robot.get('color', '')
                ])
            }]
            )
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

    spawn_robot_with_delay = TimerAction(
        period=1.0,  # Wait 1 seconds after Gazebo starts
        actions=spawn_nodes
    )
    rviz_with_delay = TimerAction(
        period=4.0,  # Wait 4 second after starting the robot_state_publisher
        actions=[rviz_node]
    )

    # create and return launch description object
    return LaunchDescription(
        [   DeclareLaunchArgument( 'world',
            default_value=[os.path.join(share_dir, 'worlds', 'barista_world.world'), ''],
            description='SDF world file'),
            gazebo,
            *rsp_nodes,
            spawn_robot_with_delay,
            rviz_with_delay
        ]
    )
