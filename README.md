# Tello ROS Wrapper

## Introduction

This is a simple ROS package that enables one to use ROS2 to command a DJI
Tello drone. This package makes use of the python library `tellopy` and thus creating
a bridge between ROS and `tellopy`.

Currently this package is a **work in progress** and from now it only allows
one to land, takeoff, perform flips and move the drone
around using velocity commands via ROS topics. In addition, the data received from
from the drone, such as the battery percentage, is also being published to a topic.

## What can you expect from this package

For the future, I plan to integrate the rest of tellopy features into this package.
