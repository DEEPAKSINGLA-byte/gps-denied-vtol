"""Launch camera image segmentation and its ROS output topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the segmentation node with configurable camera and model."""
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('model', default_value='yolov8n-seg.pt'),
        Node(
            package='quadcopter_perception',
            executable='segment_camera',
            name='quadcopter_segmentation',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'model': LaunchConfiguration('model'),
            }],
        ),
    ])
