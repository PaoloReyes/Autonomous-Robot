import os
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description() -> LaunchDescription:

    manchester_camera_launch = ExecuteProcess(cmd=['ros2', 'launch', 'ros_deep_learning', 'video_source.ros2.launch.py'], output='screen')

    micro_ros_Node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyUSB0'],
    )

    return LaunchDescription([
        manchester_camera_launch,
        micro_ros_Node
    ])