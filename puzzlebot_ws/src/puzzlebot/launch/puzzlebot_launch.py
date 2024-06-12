import os
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description() -> LaunchDescription:

    manchester_camera_launch_path = os.path.join(get_package_share_directory('ros_deep_learning'), 'launch', 'video_source.ros2.launch')
    
    micro_ros_Node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyUSB0'],
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manchester_camera_launch_path),
        ),
        micro_ros_Node
    ])