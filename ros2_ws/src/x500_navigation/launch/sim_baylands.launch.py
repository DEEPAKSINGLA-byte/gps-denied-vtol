#!/usr/bin/env python3
"""Baylands simulation: x500 with a 2D lidar, SLAM and Nav2 in PX4's park world.

    ros2 launch x500_navigation sim_baylands.launch.py

Sets the Baylands defaults and includes sim_px4_lidar.launch.py (which
documents everything that is started). Any argument can be changed on the
command line, e.g. headless:=true rviz:=false follow:=true altitude:=3
max_speed:=0.5 pose:="0,0,0,0,0,0" (PX4's open-field spawn).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Middle of the orchard-like grove in the west of the park: rows of trees about
# 10 m apart, 15 trees within 20 m, nearest trunk ~7 m, at least 3.5 m clear of
# any trunk, branch or leaf. Trees are 3-4.6 m tall with leaves from ~1.2 to
# 3 m, so at the default 2 m flying height the lidar cuts through every crown.
# Found from the Baylands meshes (bark/leaf/branch vertices). Ground here is
# at z = -0.4 m, so spawning at z = 0 drops the drone a few cm onto the grass.
GROVE_POSE = '-80.7,93.2,0.0,0,0,0'


def generate_launch_description():
    share = get_package_share_directory('x500_navigation')
    defaults = {
        'world': 'baylands',
        'world_sdf': '',                 # PX4's own baylands.sdf
        'pose': GROVE_POSE,
        'altitude': '2.0',
        'max_speed': '1.0',
        'camera_offset': '-2.5,-2.5,2.5',
        'nav2_params': os.path.join(share, 'config', 'nav2_baylands.yaml'),
        'slam_params': os.path.join(share, 'config', 'slam_baylands.yaml'),
        'rviz_config': os.path.join(share, 'rviz', 'sim_baylands.rviz'),
    }
    # DeclareLaunchArgument only sets a value the command line did not give.
    return LaunchDescription([
        *(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'sim_px4_lidar.launch.py'))),
    ])
