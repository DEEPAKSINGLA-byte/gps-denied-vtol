import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Parameter bridge between Gazebo Harmonic and ROS 2
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/floorplan/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            (
                '/world/floorplan/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan',
                '/scan',
            )
        ],
        output='screen',
    )

    # 2. TF broadcaster node (PX4 odom -> base_footprint)
    tf2_node = Node(
        package='x500_px4_bridge',
        executable='tf2_broadcaster',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 3. MicroXRCEAgent for PX4 uORB telemetry over UDP
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
    )

    # 4. Static transform from base_footprint to lidar link
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.12', '0', '0.26', '0', '0', '0', 'base_footprint', 'link'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        micro_xrce_agent,
        bridge_node,
        tf2_node,
        static_tf_node,
    ])
