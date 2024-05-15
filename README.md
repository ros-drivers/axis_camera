# AXIS ROS2 driver

This ROS2 package is based on the [ROS1 noetic driver](https://github.com/ros-drivers/axis_camera/tree/noetic-devel).<br/>

Fully written in Python, this driver is under active development.<br/>

Camera used: **AXIS Q1798-LE**


## Camera preparation

Before using the ROS driver you should ensure your camera is properly connected to the PC and powered as-per the manufacturer's specifications.<br/>

We recommend configuring the camera to use a static IP address on your robot's internal wired LAN, rather than DHCP. Because the driver addresses the camera by hostname or IP address it's easier if the address is constant.

## HTTP Authentication and Anonymous Control

By default most Axis cameras require HTTP authentication to view the camera data & to send PTZ (or other) commands.<br/>

There are two solutions to this:
1. Log into the camera's web GUI and enable `Anomymous Viewers` and `Anonymous PTZ Operators`. These options can usually be found under settings > Users
2. Configure the node parameters to use a valid Axis user's username and password. This is done with the `username` and `password` arguments. Most modern Axis cameras used HTTP Digest authorization. If your camera only supports basic HTTP authentication, set the `encrypt_password` argument to false.

## Build the AXIS ROS2 driver

Below are the commands to build the ROS2 driver package:

`source ros/humble/setup.bash`<br/>
`mkdir -p axis_ws/src`<br/>
`cd axis_ws/src`<br/>
`git clone <git_link>`<br/>
`cd axis_ws/ && colcon build`<br/>
`source axis_ws/install/setup.bash`

## Usage of the AXIS ROS2 driver

If you want the rqt visualization, you must launch rqt before the node. Otherwise, the subscription to the topic will generate an image transport error that is currently not resolved.

`ros2 run axis_camera axis_camera_node --ros-args -p hostname:=hostname -p username:=username -p password:=password -p use_encrypted_password:=use_encrypted_password -p width:=width -p height:=height -p fps:=fps -p camera:=camera`