Change history
==============

0.2.0 (2015-05-06)
------------------
* Merge pull request `#35 <https://github.com/ros-drivers/axis_camera/issues/35>`_ from pal-robotics-forks/support_axis212ptz
  Added support for Axis 212 PTZ.
* Merge pull request `#29 <https://github.com/ros-drivers/axis_camera/issues/29>`_ from negre/master
  handle encrypted password authentication
* Added support for Axis 212 PTZ.
  Also made the exception when something goes wrong in the state grabber clearer.
* Merge pull request `#34 <https://github.com/ros-drivers/axis_camera/issues/34>`_ from CreativeEntropy/patch-1
  Create LICENSE file (New BSD)
* Create LICENSE (New BSD)
  Create LICENSE file to make copyright clear.
* Merge pull request `#31 <https://github.com/ros-drivers/axis_camera/issues/31>`_ from clearpathrobotics/jeff-o-patch-1
  Update axis.launch
* Update axis.launch
  Corrects an issue where a topic subscribes and publishes to the same node (axis/republish).
* handle encrypted password authentication
* Contributors: Jeff Schmidt, Julian Schrittwieser, Mike Purvis, Sammy Pfeiffer, amaury

0.1.0 (2014-07-31)
------------------

 * Hydro and Indigo release.
 * Several bugfixes and general tidyup.
 * Rename **compressed** topic **image_raw/compressed** (`#5`_).
 * Convert to catkin (`#12`_).

0.0.2 (2013-04-10)
------------------

 * Fuerte update.
 * Add **frame_id** parameter (fixes `#8`_)
 * Add camera_info_manager support (`#10`_). Adds a new dependency on
   **camera_info_manager_py**, and a new **camera_info_url**
   parameter.
 * Add some additional PTZ control nodes: teleop.py, teleop_twist.py,
   axis_twist.py, axis_all.py.
 * Add PTZ transform publisher: publish_axis_tf.py.

0.0.1 (2012-12-05)
------------------

 * Fuerte release.
 * Initial axis_camera package.

.. _`#5`: https://github.com/clearpathrobotics/axis_camera/issues/5
.. _`#8`: https://github.com/clearpathrobotics/axis_camera/issues/8
.. _`#10`: https://github.com/clearpathrobotics/axis_camera/issues/10
.. _`#12`: https://github.com/clearpathrobotics/axis_camera/issues/12
