# Tello ROS Wrapper

## Introduction

This is a simple ROS package that enables one to use ROS2 to command a DJI
Tello drone using command velocity. This package makes use of the python library `tellopy` and thus creating
a bridge between ROS and `tellopy`.

> **Make sure to install tellopy from source, as the current version on pip is outdated.
> You can use `install_tellopy.sh` script which will install it.**

## Ros Messages

There are 2 types of messages available. The `FlightData.msg` contains the all the data that comes from the drone such as the battery percentage.
The second message type is `FlipControl.msg` which is used to signal the drone to flip in the different directions.

## Ros Topics

> All topics used by the package start with `/tello/`.

### Subscribed Topics

- `/takeoff`(std_msgs/Empty): If any message is sent to this topic, even emtpy, will trigger the drone to takeoff.
- `/land`(std_msgs/Empty): If any message is sent to this topic, even emtpy, will trigger the drone toland.
- `/cmd_vel`(std_msgs/Twist): Command velocity commands sent to this topic will make the drone move if it is in the air.
- `/flip`(tello_msgs/FlipControl): If any of the flip directions are set to true, it will make the drone flip on that same direction.
- `/palm_land`(std_msgs/Empty): Activates the palm_land feature. (Not tested)
- `/throw_and_go`(std_msgs/Empty): Activates the throw_and_go feature. (Not implemented yet)
- `/set_att_limit`(std_msgs/Int32): Sets the altitude limit for the drone. (Not implemented yet)

### Published Topics

- `/tello/FlightData`(tello_msgs/FlighData): Publishes all the data coming from the drone.
- `/tello/camera/image_raw`(/sensor_msgs/Image): Publishes the camera images from the drone.
