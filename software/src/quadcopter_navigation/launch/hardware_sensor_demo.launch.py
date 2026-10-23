"""Launch camera segmentation and 2D LiDAR SLAM for real sensors."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start segmentation and SLAM; sensor drivers and TF come from the user."""
    default_slam_params = PathJoinSubstitution([
        FindPackageShare('quadcopter_navigation'),
        'config',
        'slam_toolbox_hardware.yaml',
    ])
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('slam_toolbox'),
            '/launch/online_async_launch.py',
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': LaunchConfiguration('slam_params_file'),
        }.items(),
    )
    segmentation = Node(
        package='quadcopter_perception',
        executable='segment_camera',
        name='quadcopter_segmentation',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('camera_image_topic'),
            'model': LaunchConfiguration('segmentation_model'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_image_topic',
            default_value='/camera/image_raw',
            description='ROS Image topic published by the real camera driver.',
        ),
        DeclareLaunchArgument(
            'segmentation_model',
            default_value='yolov8n-seg.pt',
            description='Ultralytics segmentation weights path or model name.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params,
            description='SLAM Toolbox parameters; edit frames for the sensor rig.',
        ),
        segmentation,
        slam_launch,
    ])
