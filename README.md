axis_camera [![Build Status](https://travis-ci.com/ros-drivers/axis_camera.svg?branch=master)](https://travis-ci.com/ros-drivers/axis_camera)
=============================================================================================================================================

Overview
--------

This [ROS](http://ros.org) package provides an [Axis network camera](http://www.axis.com/products/video/camera/index.htm) driver, written
in Python.

ROS wiki documentation: [axis_camera](http://ros.org/wiki/axis_camera)

This driver is under active development.  Its ROS interfaces are
relatively stable, but may still change.  

There is no released code API.

:warning:**Warning**
> The master branch normally contains code being tested for the next
> ROS release.  It does not always work with previous ROS distributions.
> Sometimes, it may not work at all.

Each official release is tagged in the repository. The
[change history](https://github.com/clearpathrobotics/axis_camera/blob/master/CHANGELOG.rst) describes every version.


Supported Cameras
------------------

The following is a list of cameras that have been tested with this driver and are known to work.  Other cameras may
also be usable, but have not been tested by the developers/maintainers of this package.

- [M30 Series](https://www.axis.com/en-ca/products/axis-m30-series)
- [M42 Series](https://www.axis.com/en-ca/products/axis-m42-series)
- [Q62 Series](https://www.axis.com/en-ca/products/axis-q62-series)
- [F Series](https://www.axis.com/en-ca/products/axis-f-series)


Camera Preparation
-------------------

Before using the ROS driver you should ensure your camera is properly connected to the PC and powered as-per the
manufacturer's specifications.

We recommend configuring the camera to use a static IP address on your robot's internal wired LAN, rather than DHCP.
Because the driver addresses the camera by hostname or IP address it's easier if the address is constant.

Some cameras work best if you also enable `Anonymous Viewing` and `Anonymous PTZ Commands`, if supported.  This removes
the need to authenticate to the camera to consume video data or control the PTZ position.


Usage
------

Once the camera is configured, simply launch the driver:

```bash
roslaunch axis_camera axis.launch hostname:=192.168.0.90 username:=root password:=password
```

If your camera supports PTZ control, you can enable it with

```bash
roslaunch axis_camera axis.launch hostname:=192.168.0.90 username:=root password:=password enable_ptz:=true
```

In the case of the F Series cameras, multiple cameras can be connected to a single controller box.  In this case, launch
the driver once for each physical camera, specifying the camera name & ID number.  The ID number corresponds to the
physical port in the e.g. F34 controller (1-4).

```bash
roslaunch axis_camera axis.launch hostname:=192.168.0.90 username:=root password:=password camera_name:=front_camera camera:=1

roslaunch axis_camera axis.launch hostname:=192.168.0.90 username:=root password:=password camera_name:=rear_camera camera:=2
```

The Q62 Series cameras also feature a night-vision mode (adds and IR illuminator and disables the IR filter), a lens
wiper, and a defogger in addition to the normal PTZ control.  To enable all of this camera's supported features, use

```bash
roslaunch axis_camera axis.launch hostname:=192.168.0.90 username:=root password:=password enable_ptz:=true enable_ir:=true enable_defog:=true enable_wiper:=true
```

Topics and Services
--------------------

The camera's main image data is published on `/camera_name/image_raw` as a `sensor_msgs/Image` message.

If the `enable_theora` argument is `true` then a the compressed camera data is also available on `/camera_name/image_raw_out`.

PTZ control (if enabled) uses the `axis_camera/Axis.msg` type:

```
float32 pan
float32 tilt
float32 zoom
float32 focus
float32 brightness
float32 iris
bool autofocus
```

To write to the camera, use

```bash
rostopic pub /camera_name/cmd axis_camera/Axis "{pan: 45.0, tilt: 20.0, zoom: 1000.0, focus: 0.0, brightness: 1.0, iris: 1.0, autofocus: true}" -1
```

All writable camera properties are set simultaneously.  It is recommended to read the camera's current state from
`/camera_name/state`, copy the `focus`, `autofocus`, `brightness`, and `iris` parameters, and then set the `pan`,
`tilt` and `zoom` fields as desired.

`pan` and `tilt` are expressed in degrees (for ease of use with Axis' REST API) with positive tilt being upwards and
positive pan being anticockwise.

`zoom` is a value from 1 to 10000, with higher numbers indicating a narrower field of view.

The Q62 Series' IR mode can be toggled by running

```bash
rosservice call /camera_name/set_ir_on "data: true"  # or "data: false"
```

When IR mode is on the IR illuminator will be turned on and the IR filter turned off.  The current state of the IR
mode can be read from `/camera_name/ir_on` as a `std_msgs/Bool`.

To enable the defogger, run

```bash
rosservice call /camera_name/set_defog_on "data: true"  # or "data: false"
```

The current state of the defogger can be read from `/camera_name/defog_on` as a `std_msgs/Bool`.

To start the lens wiper, run

```bash
rosservice call /camera_name/set_wiper_on "data: true"
```

The wiper will run for 10s and stop automatically.  You can stop the wiper early by running

```bash
rosservice call /camera_name/set_wiper_on "data: false"
```

The current state of the wiper can be read from `/camera_name/wiper_on` as a `std_msgs/Bool`.
