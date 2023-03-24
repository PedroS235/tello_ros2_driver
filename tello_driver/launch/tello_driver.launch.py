import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("tello_driver")
    param_file = os.path.join(pkg_dir, "config", "params.yaml")

    tello_driver_cmd = Node(
        package="tello_driver",
        executable="tello_driver",
        parameters=[param_file],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(tello_driver_cmd)

    return ld
