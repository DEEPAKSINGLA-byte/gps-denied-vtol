import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file')
    pi_ip = LaunchConfiguration('pi_ip', default='10.184.3.36')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('x500_navigation'),
            'config', 'nav2_real.yaml'
        ),
        description='Full path to Nav2 params file'
    )
    declare_pi_ip = DeclareLaunchArgument(
        'pi_ip', default_value='10.184.3.36',
        description='Raspberry Pi IP address for telemetry/command UDP'
    )

    # 1. MAVLink telemetry -> ROS 2 /odom + TF bridge
    mavlink_bridge_node = Node(
        package='gps_denied_bridge',
        executable='mavlink_to_ros2_bridge',
        name='mavlink_to_ros2_bridge',
        output='screen',
        parameters=[{
            'listen_port': 14551,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_rate_hz': 10.0,
            'optical_flow_scale': 1.0,
            'z_source': 'rangefinder',
        }],
    )

    # 2. LiDAR UDP receiver -> /scan LaserScan
    lidar_receiver_node = Node(
        package='gps_denied_bridge',
        executable='lidar_udp_receiver',
        name='lidar_udp_receiver_node',
        output='screen',
        parameters=[{
            'listen_port': 14552,
            'frame_id': 'laser_frame',
            'scan_topic': 'scan',
        }],
    )

    # 3. Static transform: base_link -> laser_frame (YDLIDAR mount offset)
    # Adjust x/y/z/yaw to match your physical lidar mounting.
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0.12', '0', '0.26', '0', '0', '0', 'base_link', 'laser_frame'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 4. Nav2 bringup (costmaps, planner, controller, BT navigator)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )

    # 5. ROS 2 /cmd_vel -> MAVLink commands to Pixhawk (via Pi UDP)
    cmd_bridge_node = Node(
        package='gps_denied_bridge',
        executable='mavlink_cmd_bridge',
        name='mavlink_cmd_bridge',
        output='screen',
        parameters=[{
            'cmd_vel_topic': '/cmd_vel',
            'pi_ip': pi_ip,
            'discrete_port': 14552,
            'velocity_port': 14553,
            'cmd_timeout_s': 0.50,
            'max_xy_speed_mps': 0.5,
            'auto_arm': False,
            'auto_guided': True,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_pi_ip,
        mavlink_bridge_node,
        lidar_receiver_node,
        static_tf_lidar,
        nav2_launch,
        cmd_bridge_node,
    ])
