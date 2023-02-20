# Tello ROS Wrapper

## Introduction

This is a simple ROS package that enables one to use ROS2 to command a DJI
Tello drone using command velocity. This package makes use of the python library `tellopy` and thus creating
a bridge between ROS and `tellopy`.

## Ros Messages

There are 2 types of messages available. The `FlightData.msg` contains the all the data that comes from the drone such as the battery percentage.
The second message type is `FlipControl.msg` which is used to signal the drone to flip in the different directions.

## Ros Topics

> All topics used by the package start with `/tello/`.

### Subscribed Topics

- `/tello/takeoff`(std_msgs/Header): If any message is sent to this topic, even emtpy, will trigger the drone to takeoff.
- `/tello/land`(std_msgs/Header): If any message is sent to this topic, even emtpy, will trigger the drone toland.
- `/tello/cmd_vel`(std_msgs/Twist): Command velocity commands sent to this topic will make the drone move if it is in the air.
- `/tello/flip`(tello_msgs/FlipControl): If any of the flip directions are set to true, it will make the drone flip on that same direction.

### Published Topics

- `/tello/FlightData`(tello_msgs/FlighData): Publishes all the data coming from the drone.
- `/tello/camera/image_raw`(/sensor_msgs/Image): Publishes the camera images from the drone.
