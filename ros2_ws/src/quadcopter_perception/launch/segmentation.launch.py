"""Run the YOLO + SAM segmentation node on /camera/image_raw.

    ros2 launch quadcopter_perception segmentation.launch.py
    ros2 launch quadcopter_perception segmentation.launch.py sam_imgsz:=768        # faster
    ros2 launch quadcopter_perception segmentation.launch.py sam_model:=none       # YOLO only
    ros2 launch quadcopter_perception segmentation.launch.py \
        model:=/path/yolov8s-seg.pt classes:="['person']"
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
                              description='YOLO weights path (or a name Ultralytics downloads)'),
        DeclareLaunchArgument('device', default_value='', description="'' = auto (cuda:0)"),
        DeclareLaunchArgument('conf', default_value='0.35'),
        DeclareLaunchArgument('imgsz', default_value='640'),
        DeclareLaunchArgument('classes', default_value="['']",
                              description="Class names to keep, e.g. \"['person']\"; [''] = all"),
        DeclareLaunchArgument('sam_model', default_value='sam2.1_t.pt',
                              description='SAM weights (sam2.1_t/s/b.pt, mobile_sam.pt); '
                                          'none = off'),
        DeclareLaunchArgument('sam_imgsz', default_value='1024',
                              description='SAM input size; 768 is ~2x faster'),
        DeclareLaunchArgument('sam_max_boxes', default_value='8'),
        DeclareLaunchArgument('sam_min_score', default_value='0.5'),
        DeclareLaunchArgument('sam_min_agreement', default_value='0.3'),
        DeclareLaunchArgument('sam_every_n', default_value='1',
                              description='Run SAM on every Nth frame (others use YOLO masks)'),
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
            'sam_model': ParameterValue(L('sam_model'), value_type=str),
            'sam_imgsz': ParameterValue(L('sam_imgsz'), value_type=int),
            'sam_max_boxes': ParameterValue(L('sam_max_boxes'), value_type=int),
            'sam_min_score': ParameterValue(L('sam_min_score'), value_type=float),
            'sam_min_agreement': ParameterValue(L('sam_min_agreement'), value_type=float),
            'sam_every_n': ParameterValue(L('sam_every_n'), value_type=int),
        }],
    )
    return LaunchDescription(args + [node])
