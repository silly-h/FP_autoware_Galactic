import os
from ament_index_python import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


def launch_setup(context, *args, **kwargs):
    # set concat filter as a component
    # concat_component_raw = ComposableNode(
    #     package="pointcloud_preprocessor",
    #     plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerOusterComponent",
    #     name="concatenate_data_raw",
    #     remappings=[("output", "concatenated/pointcloud_raw")],
    #     parameters=[
    #         {
    #             "input_topics": [
    #                 "/points",
    #                 # "/sensing/lidar/rear_right/ouster/points",
    #                 # "/sensing/lidar/front_right/ouster/points",
    #                 # "/sensing/lidar/rear_left/ouster/points",
    #             ],
    #             "output_frame": LaunchConfiguration("base_frame"),
    #         }
    #     ],
    #     extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    # )
    # set crop box filter as a component
    cropbox_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter",
        remappings=[
            ("input", "/points"),
            ("output", "concatenated/pointcloud"),
        ],
        parameters=[
            {
                "input_frame": "velodyne_base_link",
                "output_frame": LaunchConfiguration("base_frame"),
                "min_x": 0.05,
                "max_x": 0.05,
                "min_y": -0.05,
                "max_y": 0.05,
                "min_z": 0.0,
                "max_z": 0.0,
                "negative": True,
            }
        ],
        # qos_profile={
        #     "input": QoSProfile(
        #         reliability=QoSReliabilityPolicy.RELIABLE,
        #         history=QoSHistoryPolicy.KEEP_LAST,
        #         depth=5
        #     ),
        #     "output": QoSProfile(
        #         reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #         history=QoSHistoryPolicy.KEEP_LAST,
        #         depth=5
        #     )
        # }
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[],
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
        output="screen",
    )

    target_container = (
        container
        if UnlessCondition(LaunchConfiguration("use_pointcloud_container")).evaluate(context)
        else LaunchConfiguration("container_name")
    )

    # load concat or passthrough filter
    concat_loader = LoadComposableNodes(
        composable_node_descriptions=[cropbox_component],
        target_container=target_container,
        condition=IfCondition(LaunchConfiguration("use_concat_filter")),
    )

    return [container, concat_loader]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("base_frame", "base_link")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "False")
    add_launch_arg("use_pointcloud_container", "True")
    add_launch_arg("container_name", "pointcloud_preprocessor_container")
    add_launch_arg("use_concat_filter", "True")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    # # 添加 static_transform_publisher
    # static_transform_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher',
    #     arguments=['0.1563', '0', '0.71', '0', '0', '0', 'base_link', 'velodyne_base_link'],
    # )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        # + [static_transform_publisher]
        + [OpaqueFunction(function=launch_setup)]
    )