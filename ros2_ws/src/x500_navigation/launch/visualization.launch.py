"""RViz2 + validation metrics for the real GPS-denied stack.

Standalone (sensors/bridges already running):
    ros2 launch x500_navigation visualization.launch.py
Also included by navigation_real.launch.py (rviz:=true / metrics:=true).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_cfg = os.path.join(get_package_share_directory('x500_navigation'),
                            'rviz', 'gps_denied_real.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('metrics', default_value='true'),
        DeclareLaunchArgument('expect_camera', default_value='true'),
        DeclareLaunchArgument('expect_segmentation', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_cfg),
        Node(
            package='gps_denied_bridge', executable='metrics_monitor',
            name='metrics_monitor', output='screen',
            condition=IfCondition(LaunchConfiguration('metrics')),
            parameters=[{'expect_camera': LaunchConfiguration('expect_camera'),
                         'expect_segmentation': LaunchConfiguration('expect_segmentation'),
                         'use_sim_time': False}]),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': False}]),
    ])
