import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("tello_driver")
    param_file = os.path.join(pkg_dir, "config", "params.yaml")

    ns_launch_arg = DeclareLaunchArgument(
        "ns",
        default_value="",
        description="Namespace for the tello_driver_node",
    )

    tello_driver_cmd = Node(
        package="tello_driver",
        name="tello_driver_node",
        namespace=LaunchConfiguration("ns"),
        executable="tello_driver",
        parameters=[param_file],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(ns_launch_arg)
    ld.add_action(tello_driver_cmd)

    return ld
