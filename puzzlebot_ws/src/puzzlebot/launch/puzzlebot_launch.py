import os
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description() -> LaunchDescription:

    manchester_camera_launch = ExecuteProcess(cmd=['ros2', 'launch', 'ros_deep_learning', 'video_source.ros2.launch'], output='screen')

    micro_ros_Node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyUSB1'],
    )

    controller_node = Node(
        package='puzzlebot',
        executable='controller_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('puzzlebot'), 'config', 'controller_node_params.yaml')]
    )

    return LaunchDescription([
        manchester_camera_launch,
        controller_node,
        micro_ros_Node
    ])