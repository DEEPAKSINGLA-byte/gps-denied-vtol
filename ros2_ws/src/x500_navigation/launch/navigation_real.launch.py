import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    segmentation = LaunchConfiguration('segmentation')
    declare_segmentation = DeclareLaunchArgument(
        'segmentation', default_value='true',
        description='Run YOLOv8 segmentation on /camera/image_raw (GPU) -> /segmentation/*'
    )
    declare_seg_model = DeclareLaunchArgument(
        'seg_model', default_value='yolov8n-seg.pt',
        description='Segmentation weights (path, or a name Ultralytics downloads)'
    )
    # 768 px measured on the real drone: 36 ms inference / 46 ms capture->result vs
    # 50 / 60 ms at 1024, with near-identical masks. Use 1024 for the sharpest outlines.
    declare_seg_sam_imgsz = DeclareLaunchArgument(
        'seg_sam_imgsz', default_value='768',
        description='SAM input size for segmentation (768 = lower latency, 1024 = sharpest)'
    )
    declare_seg_sam_model = DeclareLaunchArgument(
        'seg_sam_model', default_value='sam2.1_t.pt',
        description="SAM weights for segmentation; 'none' = YOLO masks only"
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
    # Lidar is mounted directly below base_link (0.15 m down, no x/y offset).
    # Adjust yaw if the lidar is rotated relative to the airframe.
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0', '0', '-0.15', '0', '0', '0', 'base_link', 'laser_frame'],
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

    # 6b. YOLOv8 segmentation on the camera topic (needs camera:=true)
    segmentation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('quadcopter_perception'),
            'launch', 'segmentation.launch.py')),
        launch_arguments={'model': LaunchConfiguration('seg_model'),
                          'sam_imgsz': LaunchConfiguration('seg_sam_imgsz'),
                          'sam_model': LaunchConfiguration('seg_sam_model')}.items(),
        condition=IfCondition(PythonExpression(
            ["'", camera, "' == 'true' and '", segmentation, "' == 'true'"])),
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
            'expect_segmentation': PythonExpression(
                ["'", camera, "' == 'true' and '", segmentation, "' == 'true'"]),
        }.items(),
    )

    # 8. 3D map: stacks 2D lidar slices at the drone's height into /map_3d (PointCloud2).
    #    Saved as ~/drone_maps/map3d_*.ply on shutdown or via the /map_3d/save service.
    declare_map3d = DeclareLaunchArgument('map3d', default_value='true',
                                          description='Build a 3D point cloud from lidar + height')
    map3d_node = Node(
        package='gps_denied_bridge',
        executable='scan_to_3d',
        name='scan_to_3d',
        output='screen',
        condition=IfCondition(LaunchConfiguration('map3d')),
        parameters=[{
            'map_frame': 'odom',
            'voxel_size': 0.05,
            'min_hits': 2,
            'max_range': 8.0,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        declare_camera,
        declare_segmentation,
        declare_seg_model,
        declare_seg_sam_imgsz,
        declare_seg_sam_model,
        declare_rviz,
        declare_metrics,
        declare_use_sim_time,
        declare_params_file,
        declare_pi_ip,
        declare_map3d,
        mavlink_bridge_node,
        lidar_receiver_node,
        static_tf_lidar,
        nav2_launch,
        cmd_bridge_node,
        camera_node,
        segmentation_launch,
        static_tf_camera,
        viz_launch,
        map3d_node,
    ])
