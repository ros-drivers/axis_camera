Change history
==============

0.1.0 (forthcoming)
-------------------

 * Groovy and Hydro release.
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
