from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tello_driver_cmd = Node(
        package="tello_driver",
        executable="tello_driver",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(tello_driver_cmd)

    return ld
