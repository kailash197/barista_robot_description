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

    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "xacro", urdf_file)

    robot_1 = {'name': 'morty', 'color':'Blue'}
    robot_2 = {'name': 'rick', 'color': 'Red'}
    
    rsp_robot_1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_1.get("name"),
        parameters=[{'frame_prefix': robot_1.get("name") +'/', 
                     'use_sim_time': True,
                     'robot_description': Command(['xacro ', robot_desc_path, 
                                                   ' robot_name:=', robot_1.get('name'),
                                                   ' color:=', robot_1.get('color'),
                                                    ' link_joint_prefix:=', robot_1.get("name") +'/'
                                                   ' include_laser:=true'])}],
        output="screen"
    )
    rsp_robot_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_2.get("name"),
        parameters=[{'frame_prefix': robot_2.get("name") +'/', 
                     'use_sim_time': True,
                     'robot_description': Command(['xacro ', robot_desc_path, 
                                                   ' robot_name:=', robot_2.get('name'),
                                                   ' color:=', robot_2.get('color'),
                                                   ' link_joint_prefix:=', robot_2.get("name") +'/', 
                                                   ' include_laser:=true'])}],
        output="screen"
    )

    #gazebo
    position_xyz = [0.0, 0.0, 0.2]
    orientation_rpy = [0.0, 0.0, 0.0]
    spawn_robot_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', robot_1.get('name'),
                   '-x', str(position_xyz[0]),
                   '-y', str(position_xyz[1]),
                   '-z', str(position_xyz[2]),
                   '-R', str(orientation_rpy[0]),
                   '-P', str(orientation_rpy[1]),
                   '-Y', str(orientation_rpy[2]),
                   '-topic', robot_1.get('name')+'/robot_description'
                   ]
    )
    position_xyz = [1.0, 1.0, 0.2]
    orientation_rpy = [0.0, 0.0, 0.0]
    spawn_robot_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', robot_2.get('name'),
                   '-x', str(position_xyz[0]),
                   '-y', str(position_xyz[1]),
                   '-z', str(position_xyz[2]),
                   '-R', str(orientation_rpy[0]),
                   '-P', str(orientation_rpy[1]),
                   '-Y', str(orientation_rpy[2]),
                   '-topic', robot_2.get('name')+'/robot_description'
                   ]
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
        actions=[spawn_robot_1,spawn_robot_2]
    )
    rviz_with_delay = TimerAction(
        period=4.0,  # Wait 4 second after starting the robot_state_publisher
        actions=[rviz_node]
    )

    # create and return launch description object
    return LaunchDescription(
        [   DeclareLaunchArgument( 'world',
            default_value=[os.path.join(pkg_gazebo_ros, 'worlds', 'empty.world'), ''],
            description='SDF world file'),
            gazebo,
            rsp_robot_1,
            spawn_robot_1,
            # rviz_with_delay
        ]
    )
