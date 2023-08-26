#!/usr/bin/env python3
import rclpy
from tello_driver.tello_driver import TelloRosWrapper
import atexit

NODE_NAME = "tello_driver"


def main(args=None):
    rclpy.init(args=args)

    # - Start the node
    tello_ros_wrapper = TelloRosWrapper(NODE_NAME)

    atexit.register(tello_ros_wrapper.shutdown_rountine)

    rclpy.spin(tello_ros_wrapper)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
