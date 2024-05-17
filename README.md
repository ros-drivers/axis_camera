# AXIS ROS 2 driver

This ROS 2 package is based on the [ROS 1 noetic driver](https://github.com/ros-drivers/axis_camera/tree/noetic-devel).

Fully written in Python, this driver is under active development.

## Cameras Tested

The following cameras have been tested with this driver:

- Axis Q198-LE
- Axis Q62


## Camera preparation

Before using the ROS driver you should ensure your camera is properly connected to the PC and powered as-per the manufacturer's specifications.

We recommend configuring the camera to use a static IP address on your robot's internal wired LAN, rather than DHCP. Because the driver addresses the camera by hostname or IP address it's easier if the address is constant.

## HTTP Authentication and Anonymous Control

By default most Axis cameras require HTTP authentication to view the camera data & to send PTZ (or other) commands.

There are two solutions to this:
1. Log into the camera's web GUI and enable `Anomymous Viewers` and `Anonymous PTZ Operators`. These options can usually be found under settings > Users
2. Configure the node parameters to use a valid Axis user's username and password. This is done with the `username` and `password` arguments. Most modern Axis cameras used HTTP Digest authorization. If your camera only supports basic HTTP authentication, set the `encrypt_password` argument to false.

## Building the Driver

Below are the commands to build the package:

```bash
cd colcon_ws/src
git clone git@github.com:ros-drivers/axis_camera.git -b humble-devel
git clone git@github.com:clearpathrobotics/ptz_action_server.git -b ros2
git clone git@github.com:clearpathrobotics/camera_info_manager_py.git -b ros2
cd ..
colcon build
```

The [`ptz_action_server`](https://github.com/clearpathrobotics/ptz_action_server) package contains the `.action` definitions used for position
and velocity control of PTZ cameras.

## Usage of the AXIS ROS 2 driver

If you want the rqt visualization, you must launch rqt before the node. Otherwise, the subscription to the topic will generate an image transport error that is currently not resolved.

```bash
ros2 launch axis_camera axis_camera.launch
```

Refer to `launch/axis_camera.launch` for launch arguments.