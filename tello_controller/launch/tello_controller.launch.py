from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tello_controller_cmd = Node(
        package="tello_controller",
        executable="tello_controller",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(tello_controller_cmd)

    return ld
