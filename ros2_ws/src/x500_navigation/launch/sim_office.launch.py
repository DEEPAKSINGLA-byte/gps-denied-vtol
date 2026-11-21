#!/usr/bin/env python3
"""Indoor office simulation: x500 with a 2D lidar, SLAM and Nav2.

    ros2 launch x500_navigation sim_office.launch.py

The world is worlds/office.sdf (made by worlds/make_office_world.py): a
24 x 16 m office floor with a corridor, an open-plan area, meeting room,
kitchen, two private offices, a server room and a lobby, joined by 1.6 m
doorways. The drone starts in the lobby, flies at 1.5 m and, once airborne,
explores the whole floor on its own with explore_lite (explore:=false to fly
it by hand with RViz goals instead).

Sets these defaults and includes sim_px4_lidar.launch.py (which documents
everything that is started). Any argument can be changed on the command line,
e.g. headless:=true rviz:=false follow:=true explore:=false max_speed:=0.3.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Lobby, south-east of the floor, facing west down the corridor.
LOBBY_POSE = '8.0,-4.5,0.05,0,0,3.14159'


def generate_launch_description():
    share = get_package_share_directory('x500_navigation')
    defaults = {
        'world': 'office',
        'world_sdf': os.path.join(share, 'worlds', 'office.sdf'),
        'pose': LOBBY_POSE,
        # Desks and tables are below 1.1 m, cabinets and racks above 2 m, so
        # at 1.5 m the drone clears the low furniture and the lidar (at
        # ~1.8 m) sees walls and every tall object.
        'altitude': '1.5',
        'max_speed': '0.5',
        'camera_offset': '2.5,2.0,2.5',
        'nav2_params': os.path.join(share, 'config', 'nav2_office.yaml'),
        'slam_params': os.path.join(share, 'config', 'slam_office.yaml'),
        'rviz_config': os.path.join(share, 'rviz', 'sim_office.rviz'),
        # Explore the floor on its own (explore_lite), then fly back to the
        # lobby. explore:=false to send goals by hand in RViz instead.
        'explore': 'true',
        'explore_params': os.path.join(share, 'config', 'explore_office.yaml'),
    }
    # DeclareLaunchArgument only sets a value the command line did not give.
    return LaunchDescription([
        *(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'sim_px4_lidar.launch.py'))),
    ])
