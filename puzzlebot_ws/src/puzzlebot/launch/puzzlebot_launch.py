import os
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description() -> LaunchDescription:

    manchester_camera_launch_path = os.path.join(get_package_share_directory('ros_deep_learning'), 'launch', 'video_source.launch.py')
    
    yolv8_segmentation = Node(
        package='yolo_segmentation',
        executable='yolo_segmentation',
        name='yolo_segmentation_node',
        output='screen',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manchester_camera_launch_path),
        ),
        yolv8_segmentation
    ])