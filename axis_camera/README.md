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

If you have used this driver with a specific model of camera not listed below, please submit a PR so we can keep this
list up-to-date.

- [F Series](https://www.axis.com/products/axis-f-series)
- [M30 Series](https://www.axis.com/products/axis-m30-series)
- [P55 Series](https://www.axis.com/products/axis-p55-series)
- [Q62 Series](https://www.axis.com/products/axis-q62-series)



Camera Preparation
-------------------

Before using the ROS driver you should ensure your camera is properly connected to the PC and powered as-per the
manufacturer's specifications.

We recommend configuring the camera to use a static IP address on your robot's internal wired LAN, rather than DHCP.
Because the driver addresses the camera by hostname or IP address it's easier if the address is constant.

HTTP Authentication and Anonymous Control
------------------------------------------

By default most Axis cameras require HTTP authentication to view the camera data & to send PTZ (or other) commands.

There are two solutions to this:

1. Log into the camera's web GUI and enable `Anomymous Viewers` and `Anonymous PTZ Operators`. These options can usually
   be found under settings > Users
2. Configure the launch file to use a valid Axis user's username and password.  This is done with the `username` and
   `password` arguments to `axis.launch`.  Most modern Axis cameras used HTTP Digest authorization.  If your camera
   only supports basic HTTP authentication, set the `encrypt_password` argument to `false` in `axis.launch`.

Usage Examples
---------------

Once the camera is configured, simply launch the driver:

```bash
roslaunch axis_camera axis.launch hostname:=192.168.0.90 username:=root password:=password
```

If your camera requires basic authentication instead of digest authentication, set the `encrypt_password` argument:

```bash
roslaunch axis_camera axis.launch hostname:=192.168.0.90 username:=root password:=password encrypt_password:=false
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

The camera's main image data is published on `/camera_name/image_raw/compressed` as a `sensor_msgs/CompressedImage`.

If the `enable_theora` argument is `true` then additional image topics are available in the `/camera_name/image_raw_out`
namespace, including `/camera_name/image_raw_out/theora` as `theora_image_transport/Packet` messages.

PTZ control (if enabled) uses the `axis_camera/Axis.msg` type:

```
float32 pan
float32 tilt
float32 zoom
float32 focus
float32 brightness
float32 iris
bool autofocus
bool autoiris
```

To write to the camera, use

```bash
rostopic pub /camera_name/cmd axis_camera/Axis "{pan: 45.0, tilt: 20.0, zoom: 1000.0, focus: 0.0, brightness: 1.0, iris: 1.0, autofocus: true, autoiris: true}" -1
```

All writable camera properties are set simultaneously.  It is recommended to read the camera's current state from
`/camera_name/state`, copy the `focus`, `autofocus`, `brightness`, and `iris` parameters, and then set the `pan`,
`tilt` and `zoom` fields as desired.  Failure to set the `brightness` field may result in a very dark image.

`pan` and `tilt` are expressed in degrees (for ease of use with Axis' REST API) with positive tilt being upwards and
positive pan being clockwise.

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
