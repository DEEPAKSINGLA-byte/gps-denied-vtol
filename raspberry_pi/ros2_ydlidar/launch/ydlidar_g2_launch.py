#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ydlidar_ros2_driver')
    params_file = os.path.join(pkg_share, 'params', 'ydlidar_g2.yaml')

    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_node',
        name='ydlidar_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file]
    )

    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser4',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame']
    )

    return LaunchDescription([
        ydlidar_node,
        tf_base_to_laser
    ])
