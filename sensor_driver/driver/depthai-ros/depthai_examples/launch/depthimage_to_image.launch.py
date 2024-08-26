from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            remappings=[('depth','/stereo/depth'),
                        ('depth_camera_info','/stereo/camera_info'),
                        ('scan','/oak_scan')],
            parameters=[{
                'scan_time': 0.033,    
                'range_min': 0.45,   #投影点的最小距离单位（米），更近的被丢弃
                'range_max': 5.0,   #投影点的最大距离单位（米），更远的被丢弃
                'scan_height': 5,    #depthimage中用于转成laserscan的行
                'output_frame': 'camera_depth_optical_frame'  #发布的帧 ID
            }]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='depthimage_to_laserscan_tf',
            arguments=['0','0','0','0','0','0','1','oak-d_frame','camera_depth_optical_frame']
        )
    ])