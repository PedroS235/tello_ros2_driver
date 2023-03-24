#!/usr/bin/env python3
import rclpy
from tello_driver.tello_driver import TelloRosWrapper

NODE_NAME = "tello_driver"


def main(args=None):
    rclpy.init(args=args)

    # - Start the node
    tello_ros_wrapper = TelloRosWrapper(NODE_NAME)
    tello_ros_wrapper.begin()
    while not tello_ros_wrapper.signal_shutdown:
        rclpy.spin_once(tello_ros_wrapper)
    tello_ros_wrapper.shutdown_rountine()
    tello_ros_wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
