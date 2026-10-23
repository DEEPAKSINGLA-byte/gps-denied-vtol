from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    # Create the ros_gz_bridge node for the lidar scan
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/floorplan/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        remappings=[
            ('/world/floorplan/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan', '/scan')
        ],
        output='screen'
    )

    # NOTE: this launch file previously started a `tf2.py` odometry TF
    # publisher via ExecuteProcess, but that script was never committed to
    # the repository (see git history for nav2/). It is intentionally not
    # started here so the launch does not crash on a missing file.
    # The `odom` -> `base_footprint` transform is expected from the SLAM
    # pipeline (e.g. slam_toolbox `map` -> `odom` plus an odometry source
    # for `odom` -> `base_footprint`, such as robot_localization or a
    # Gazebo odometry republisher). Add that node here when available.
    odom_tf_missing = LogInfo(
        msg='[x500_navigation] odom -> base_footprint TF publisher not configured; '
            'provide it via slam_toolbox / robot_localization.'
    )

    # Run MicroXRCEAgent
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )

    # Static transform from base_link to link
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.12', '0', '0.26', '0', '0', '0', 'base_footprint', 'link'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        micro_xrce_agent,
        bridge_node,
        odom_tf_missing,
        static_tf_node
    ])
