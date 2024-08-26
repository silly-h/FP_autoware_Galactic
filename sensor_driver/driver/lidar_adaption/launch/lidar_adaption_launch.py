import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_adaption',
            executable='lidar_adaption',
            name='lidar_adaption_node',
            output='screen'
        ),
    ])
