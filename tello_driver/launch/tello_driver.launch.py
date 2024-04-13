import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("tello_driver")
    default_param_file = os.path.join(pkg_dir, "config", "params.yaml")

    ns_launch_arg = DeclareLaunchArgument(
        "ns",
        default_value="",
        description="Namespace for the tello_driver_node",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=str(default_param_file),
        description="Path to the params file",
    )

    tello_driver_node = Node(
        package="tello_driver",
        namespace=LaunchConfiguration("ns"),
        executable="tello_driver",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(params_file_arg)
    ld.add_action(ns_launch_arg)
    ld.add_action(tello_driver_node)

    return ld
