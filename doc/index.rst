axis_camera: ROS Axis PTZ cameras support
=========================================

.. toctree::
   :hidden:

   axis_camera
   axis
   axis_ptz
   changelog

Overview
========

This ROS_ package provides an `Axis network camera`_ driver, written
in Python. It provides both a ROS API (via ROS topics and parameters) and Python API (which can be used without having ROS at all).

The ROS API (topics, parameters, services) is documented on the ROS wiki: `axis_camera`_.

This driver is under active development.  Its interface has undergone a major rework towards better usability and versatility. 
This rework was, however, done with backwards compatibility in mind.
The API might still change if some problems are found.

Tested compatible ROS versions: 

- Indigo

Tested compatible Axis camera models: 

- 214 PTZ
- P5512-E
- V5914
- others should also work, because the VAPIX API should be supported on all Axis cameras

**Warning**::

  The master branch normally contains code being tested for the next
  ROS release.  It does not always work with previous ROS distributions.
  Sometimes, it may not work at all.

Each official release is tagged in the repository. The `change
history`_ describes every version.

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
