import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


# Override with `export GPS_DENIED_PI_IP=...` or `pi_ip:=...` on the command line.
DEFAULT_PI_IP = os.environ.get('GPS_DENIED_PI_IP', '10.184.3.36')


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file')
    pi_ip = LaunchConfiguration('pi_ip', default=DEFAULT_PI_IP)

    camera = LaunchConfiguration('camera')
    declare_camera = DeclareLaunchArgument(
        'camera', default_value='true',
        description='Start the RGB camera TCP receiver (/camera/image_raw, port 5000)'
    )
    declare_rviz = DeclareLaunchArgument('rviz', default_value='true',
                                         description='Open RViz2 with the validation layout')
    declare_metrics = DeclareLaunchArgument('metrics', default_value='true',
                                            description='Run /metrics/* + /diagnostics monitor')

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
        'pi_ip', default_value=DEFAULT_PI_IP,
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
            'z_source': 'auto',   # rangefinder -> MTF-01 ground distance -> EKF -> baro
            'min_flow_quality': 20,
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

    # 6. RGB camera TCP stream -> /camera/image_raw (+ latency/decode/bandwidth metrics)
    camera_node = Node(
        package='gps_denied_bridge',
        executable='camera_tcp_receiver',
        name='camera_tcp_receiver',
        output='screen',
        condition=IfCondition(camera),
        parameters=[{'port': 5000, 'frame_id': 'camera_frame',
                     'image_topic': '/camera/image_raw'}],
    )
    # Camera mount: forward-looking, 10 cm ahead / 5 cm above base_link. Adjust to match.
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=['0.10', '0', '0.05', '0', '0', '0', 'base_link', 'camera_frame'],
        condition=IfCondition(camera),
    )

    # 7. RViz2 + metrics monitor
    viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('x500_navigation'),
            'launch', 'visualization.launch.py')),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'metrics': LaunchConfiguration('metrics'),
            'expect_camera': camera,
        }.items(),
    )

    return LaunchDescription([
        declare_camera,
        declare_rviz,
        declare_metrics,
        declare_use_sim_time,
        declare_params_file,
        declare_pi_ip,
        mavlink_bridge_node,
        lidar_receiver_node,
        static_tf_lidar,
        nav2_launch,
        cmd_bridge_node,
        camera_node,
        static_tf_camera,
        viz_launch,
    ])
