from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Create the ros_gz_bridge node for the lidar scan
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/floorplan/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/x500_lidar_2d_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            ('/world/floorplan/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan', '/scan'),
            ('/model/x500_lidar_2d_0/odometry', '/odom'),
        ],
        output='screen'
    )

    odom_tf_node = Node(
        package='quadcopter_navigation',
        executable='odom_tf_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen',
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
        odom_tf_node,
        static_tf_node
    ])
