#!/usr/bin/env python3
from tello_controller.keyboard_controller import Controller
import rclpy


def main(args=None):
    rclpy.init(args=args)
    controller = Controller("tello_controller")
    controller.begin()
    rclpy.spin(controller)


if __name__ == '__main__':
    main()
