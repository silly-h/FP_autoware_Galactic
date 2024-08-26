from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='can_communication',
            executable='can_output',
            name='can_output'
        ),
        Node(
            package='can_communication',
            executable='pub_twist',
            name='pub_twist'
        ),
    ])