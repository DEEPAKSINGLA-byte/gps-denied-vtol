#!/usr/bin/env python3
"""Launch PX4 SITL (x500) in Gazebo + gz <-> ROS 2 bridge + MicroXRCE agent.

Uses the system ``gz`` (Harmonic, 8.x) binary -- NOT ``ign`` (6.x) -- as
required by PX4 main (``px4-rc.gzsim`` enforces GZ >= 8.0.0). Nothing is
installed by this file; both simulators are expected to already exist.

Processes started:
  1. MicroXRCE-DDS agent (UDP) -- exposes PX4 uORB as /fmu/* topics.
  2. PX4 SITL ``make px4_sitl gz_x500`` with ``PX4_GZ_WORLD`` set --
     auto-starts ``gz sim`` with the selected world and spawns x500.
  3. ``ros_gz_bridge`` parameter_bridge for /clock + pose/odometry
     (mappings mirror ``config/sitl_gz_bridge.yaml``; Humble's bridge
     takes CLI mappings, so they are passed as node arguments).

Teleop is intentionally NOT launched -- run it manually afterwards::

    ros2 run quadcopter_bringup teleop
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    px4_dir_arg = DeclareLaunchArgument(
        'px4_dir',
        default_value=os.path.expanduser('~/PX4-Autopilot'),
        description='PX4-Autopilot source directory (cwd for SITL make target).',
    )
    px4_target_arg = DeclareLaunchArgument(
        'px4_target',
        default_value='gz_x500',
        description='SITL make target suffix, e.g. gz_x500.',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='baylands',
        description='Gazebo world name (baylands, forest, default, ...).',
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run gz sim server only, without GUI.',
    )
    agent_port_arg = DeclareLaunchArgument(
        'agent_port',
        default_value='8888',
        description='UDP port for the MicroXRCE-DDS agent (PX4 SITL default).',
    )

    # 1. MicroXRCE-DDS agent over UDP loopback (absolute binary path:
    #    the ros-humble micro-ros-agent wrapper is not installed).
    microxrce_agent = ExecuteProcess(
        cmd=[
            '/usr/local/bin/MicroXRCEAgent',
            'udp4',
            '-p',
            LaunchConfiguration('agent_port'),
            '-v',
        ],
        output='screen',
    )

    # 2. PX4 SITL. px4-rc.gzsim reads PX4_GZ_WORLD and starts
    #    `gz sim -r -s <world>.sdf` itself (+ GUI unless HEADLESS set).
    px4_sitl_gui = ExecuteProcess(
        cmd=['make', 'px4_sitl', LaunchConfiguration('px4_target')],
        cwd=LaunchConfiguration('px4_dir'),
        additional_env={'PX4_GZ_WORLD': LaunchConfiguration('world')},
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )
    px4_sitl_headless = ExecuteProcess(
        cmd=['make', 'px4_sitl', LaunchConfiguration('px4_target')],
        cwd=LaunchConfiguration('px4_dir'),
        additional_env={
            'PX4_GZ_WORLD': LaunchConfiguration('world'),
            'HEADLESS': '1',
        },
        output='screen',
        condition=IfCondition(LaunchConfiguration('headless')),
    )

    # 3. gz -> ROS 2 bridge (CLI mappings; Humble 0.244 has no yaml config).
    #    If a gz topic is absent the bridge just waits -- safe to over-list.
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/model/x500/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/x500/pose@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        px4_dir_arg,
        px4_target_arg,
        world_arg,
        headless_arg,
        agent_port_arg,
        microxrce_agent,
        px4_sitl_gui,
        px4_sitl_headless,
        gz_bridge,
    ])
