"""Run the YOLOv8 segmentation node on /camera/image_raw.

    ros2 launch quadcopter_perception segmentation.launch.py
    ros2 launch quadcopter_perception segmentation.launch.py model:=/path/yolov8s-seg.pt classes:="['person']"
"""
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('model', default_value='yolov8n-seg.pt',
                              description='Weights path (or a name Ultralytics downloads)'),
        DeclareLaunchArgument('device', default_value='', description="'' = auto (cuda:0)"),
        DeclareLaunchArgument('conf', default_value='0.35'),
        DeclareLaunchArgument('imgsz', default_value='640'),
        DeclareLaunchArgument('classes', default_value="['']",
                              description="Class names to keep, e.g. \"['person']\"; [''] = all"),
    ]
    L = LaunchConfiguration
    node = Node(
        package='quadcopter_perception', executable='segmentation_node',
        name='segmentation_node', output='screen',
        parameters=[{
            'image_topic': L('image_topic'),
            'model': L('model'),
            'device': L('device'),
            'conf': ParameterValue(L('conf'), value_type=float),
            'imgsz': ParameterValue(L('imgsz'), value_type=int),
            'classes': ParameterValue(L('classes'), value_type=List[str]),
        }],
    )
    return LaunchDescription(args + [node])
