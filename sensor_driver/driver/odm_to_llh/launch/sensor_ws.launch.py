from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution




def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', '/home/orin/sensor_ws/autoware_lidar_time.py'],
            output='screen'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                '/home/orin/sensor_ws/src/rslidar_sdk/launch/start.py'
            )
        ),

        Node(
            package='rs_to_velodyne',
            executable='rs_to_velodyne',
            output='screen'
        ),
    ])