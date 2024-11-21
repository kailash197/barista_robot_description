#! /usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    static_tf_pub_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_turtle_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', '/rick/odom']
    )
    static_tf_pub_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_turtle_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0.5', '0.5', '0', '0', '0', '0', 'world', '/morty/odom']
    )

    return LaunchDescription(
        [
            static_tf_pub_1,
            static_tf_pub_2
        ]
    )