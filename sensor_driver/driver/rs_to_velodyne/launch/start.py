from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        Node(namespace='rs_to_velodyne', 
        package='rs_to_velodyne', 
        executable='rs_to_velodyne', 
        output='screen'),
    ])
