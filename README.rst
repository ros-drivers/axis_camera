Overview
========

This ROS_ package provides an `Axis network camera`_ driver, written
in Python. It provides both a ROS API (via ROS topics and parameters) and Python API (which can be used without having ROS at all).

ROS wiki documentation: `axis_camera`_.

This driver is under active development.  Its interface has undergone a major rework towards better usability and versatility. 
This rework was, however, done with backwards compatibility in mind (not very well tested, though).
The reworked code's API might still change if some problems are found.

Tested compatible ROS versions: 

- Indigo

Tested compatible Axis camera models: 

- 214 PTZ 
- others should also work, because the VAPIX API should be supported on all Axis cameras

**Warning**::

  The master branch normally contains code being tested for the next
  ROS release.  It does not always work with previous ROS distributions.
  Sometimes, it may not work at all.

Each official release is tagged in the repository. The `change
history` _describes every version.

ROS usage
---------

To use this driver in ROS:

- Make sure the *axis_camera* package is located in some directory in your workspace.
- Build your workspace (using ``catkin_make`` or other build tools).
- Verify the package can be found by ROS, e.g. by ``rospack find axis_camera``. If it is not found, issue ``rospack profile`` and try to find it again. If it is still not found, the package is not built correctly or it is not inside your active workspace.
- To get the basic driver (providing video transmission and video stream control), run ``roslaunch axis_camera axis.launch hostname:=IP_OF_CAMERA``. Replace ``IP_OF_CAMERA`` with the IP address of your camera.

  - The camera stream is available at topic ``axis/image_raw/compressed`` and can be viewed e.g. in RViz or by *image_view*: ``rosrun image_view image_view image:=axis/image_raw compressed``. The topic's name *image_raw* is confusing, because it serves compressed images, but to retain backwards compatibility with the old driver version, it hasn't been changed. 
  - There is an ``axis/take_snapshot`` service of type ``axis_camera/TakeSnapshot`` which returns a still image on request.
  - The video stream parameters can be changed via *dynamic_recofigure*. Every parameter change causes all possibly open video streams to reconnect to get a new video stream with the updated parameters. The reconfigurable parameters are:
  
    - *use_color* (bool): If true, color image is streamed, otherwise only grayscale.
    - *compression* (int): The lossy compression level. Zero means no compression (best image quality, high data rates), 100 is the maximum compression (worst image quality, visible artifacts, low data rates). Default: 0.
    - *fps* (int): The desired number of frames per second to be transmitted. Allowed values are 1-30. Default: 24.
    - *resolution* (string enum): Allowed values are the CIF_ image resolution names. Currently supported are (sorted in decreasing order by image width in pixels): 16CIF, 4CIF, 2CIFEXP, 4SIF, 2CIF, CIF, SIF, SCIF, QCIF, SQCIF. Default: 4CIF.
    - *use_square_pixels* (bool): By default, the CIF resolutions have pixels with aspect ratio of 11:12. IF you set this property to true, the video stream is going to be strechted horizontally so that the pixels are square. This of course changes the physical resolution of the image (e.g. the 4CIF resolution, 704x576, becomes 768x576 after the stretching). Default: true.
    
  - To get the image stream either in *raw* or in *theora* formats, run the driver with ``enable_theora:=1`` parameter. Then subscribe to topic ``axis/image_raw_out/raw`` or ``axis/image_raw_out/theora``.
    
- If you also intend to control the camera using a PTZ unit, run the driver with the parameter *enable_ptz* set to 1: ``roslaunch axis_camera axis.launch enable_ptz:=1 hostname:=IP_OF_CAMERA``. 

  - With the PTZ control running, you get the possibility to read the current camera position:
  
    - As a *sensor_msgs/JointStates* message at topic ``axis/camera/joint_states`` (joint names are ``axis_pan_j`` and ``axis_tilt_j``).
    - As an *axis_camera/PTZ* message at topic ``axis/camera/ptz``.
    - TF messages are also published, with frame names ``axis_stand`` (as the base of the model), ``axis_pan_head`` and ``axis_tilt_head``. Thanks to the TF messages, you can display the camera model in RViz.
    
  - To control the pan, tilt and zoom, there are several options. Be aware that the physical execution may take some time.
  
    - Topic ``axis/control/pan_tilt_zoom/absolute`` (type ``axis_camera/PTZ``): Command the PTZ unit with an absolute pose.
    - Topic ``axis/control/pan_tilt_zoom/relative`` (type ``axis_camera/PTZ``): Command the PTZ unit with a relative pose shift.
    - Topic ``axis/control/pan_tilt_zoom/velocity`` (type ``axis_camera/PTZ``): Command the PTZ unit velocity in terms of pan, tilt and zoom.
    - Similarly, there are topics ``axis/control/pan_tilt/absolute``, ``axis/control/pan_tilt/relative``, ``axis/control/pan_tilt/velocity`` of type ``axis_camera/PTZ``, where the zoom component is ignored.
    - There are also topics ``axis/control/pan/absolute``, ``axis/control/pan/relative``, ``axis/control/pan/velocity`` (respectively for tilt and zoom, too) of type ``std_msgs/Float32`` (pan, tilt) and ``std_msgs/Int32`` (zoom).
    - Topic ``axis/control/look_at`` (type ``axis_camera/PointInRectangle``) can be used to process user clicks inside an image. To the message, x/y coordinates are given, and also the pixel width/height of the image, and the camera is moved so that the point clicked at (x/y) will be the new optical center.
    
  - Other features of the camera can also be controled when the PTZ control is active:
  
    - Focus:
    
      - Topic ``axis/control/camera/focus/absolute`` (type ``std_msgs/Float32``): Set an absolute focus value (0-9999).
      - Topic ``axis/control/camera/focus/relative`` (type ``std_msgs/Float32``): Adjust the focus value by the given amount (0-9999).
      - Topic ``axis/control/camera/focus/velocity`` (type ``std_msgs/Float32``): Set focus change velocity, active until another focus command is issued (0-9999).
      - Topic ``axis/control/camera/focus/auto`` (type ``std_msgs/Bool``): Set the autofocus feature. If on, setting the focus value has no effect.
      
    - Iris:
    
      - Topic ``axis/control/camera/iris/absolute`` (type ``std_msgs/Float32``): Set an absolute iris value (0-9999).
      - Topic ``axis/control/camera/iris/relative`` (type ``std_msgs/Float32``): Adjust the iris value by the given amount (0-9999).
      - Topic ``axis/control/camera/iris/velocity`` (type ``std_msgs/Float32``): Set iris change velocity, active until another focus command is issued (0-9999).
      - Topic ``axis/control/camera/iris/auto`` (type ``std_msgs/Bool``): Set the autoiris feature. If on, setting the focus value has no effect. Having autoiris on is needed for backlight compensation and for auto IR cut filter.
      
    - Brightness:
    
      - Topic ``axis/control/camera/brightness/absolute`` (type ``std_msgs/Float32``): Set an absolute brightness value (0-9999).
      - Topic ``axis/control/camera/brightness/relative`` (type ``std_msgs/Float32``): Adjust the brightness value by the given amount (0-9999).
      - Topic ``axis/control/camera/brightness/velocity`` (type ``std_msgs/Float32``): Set brightness change velocity, active until another focus command is issued (0-9999).

    - Backlight compensation:
    
      - Topic ``axis/control/camera/backlight_compensation`` (type ``std_msgs/Bool``): Use backlight compensation (requires autoiris on).
      
    - Infrared cut filter:

      - Topic ``axis/control/camera/ir_cut_filter/auto`` (type ``std_msgs/Bool``): Automatically turn on/off the IR cut filter as it is needed (requires autoiris on).
      - Topic ``axis/control/camera/ir_cut_filter/use`` (type ``std_msgs/Bool``): If auto IR cut filter is off, this property sets the filter on or off. If the filter is set to auto, setting this property has no effect.

Python usage
------------

High-level API
**************

In module ``axis_camera``, there are the classes ``ImageStreamingThread``, ``PositionStreamingThread`` and ``AxisCameraController``. Normally, they are started by the ROS nodes, but you can start them manually.

Unfortunately, because of backwards compatibility, these classes take a node instance in their constructors, so you either have to use them with the ROS nodes running, or you could pass a duck-typed class with the required members.

A future refactoring will probably completely split these threads from the ROS nodes.

Low-level API
*************

Class ``axis_camera.VAPIX`` provides a direct interface to the camera's HTTP VAPIX API. It is built so that it simplifies creating API commands, detects some error states and handles them.

The class provides autodetection of the VAPIX version supported by the camera (either v2 or v3). So, basically, to get an instance of the API, call ``api = VAPIX.get_api_for_camera(hostname, username, password, camera_id, use_encrypted_password)``. 

The class methods are well documented, so please refer to that documentation. In the future, this documentation should be compiled as the standard ROS package Python documentation and available through docs.ros.org .

This class can be used completely without ROS in other projects. It just uses two or three methods from the ``rospy`` module (mostly logging), so if you either edit the class or duck-type the rospy module and the required functions, you can happily use this Python interface from anywhere. In case you want to use this class without ROS, you don't need to build the package with ``catkin_make``.


.. _`Axis network camera`: http://www.axis.com/products/video/camera/index.htm
.. _`change history`: https://github.com/clearpathrobotics/axis_camera/blob/master/CHANGELOG.rst
.. _`axis_camera`: http://ros.org/wiki/axis_camera
.. _ROS: http://ros.org
.. _CIF: https://en.wikipedia.org/wiki/Common_Intermediate_Format
