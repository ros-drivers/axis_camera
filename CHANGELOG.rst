Change history
==============

0.4.3 (2022-08-22)
------------------
* Merge pull request `#73 <https://github.com/ros-drivers/axis_camera/issues/73>`_ from jhiggins-cpr/noetic-devel
  Add frames-per-second (fps) as a configurable option
* Reverting package change as it happens automatically
* Add frames-per-second (fps) as a configurable option
* Contributors: Jason Higgins, Tony Baltovski

0.4.2 (2022-07-29)
------------------
* Explicitly use Python3 in shebang for teleop nodes; remove unnecessary shebang in setup.py (`#72 <https://github.com/ros-drivers/axis_camera/issues/72>`_)
* Contributors: Joey Yang

0.4.1 (2022-06-16)
------------------
* Fix the #! lines to use python3, decode the utf8 bytes into a string to suppress a warning when parsing the camera position
* Contributors: Chris Iverach-Brereton

0.4.0 (2021-11-29)
------------------
* upgraded cmakelist and package.xml, and setup.py for noetic (`#70 <https://github.com/ros-drivers/axis_camera/issues/70>`_)
* Update the python files to be python-3 compliant.  Fix some bugs in the image data parsing needed as part of this update
* Merge pull request `#55 <https://github.com/ros-drivers/axis_camera/issues/55>`_ from sgemme-csa/master
  KeyError in publishCameraState when camera is not ready on PTZ camera
* Merge branch 'master' of github.com:ros-drivers/axis_camera
* Expose the height & width parameters as arguments in the launch file
* Merge pull request `#56 <https://github.com/ros-drivers/axis_camera/issues/56>`_ from jeff-o/patch-1
  Update axis.launch
* Revert "Fix up the main scripts to be python-3 compliant"
  This reverts commit 569e4b22415edee653914fa387a689d2e85e2879.
* Fix up the main scripts to be python-3 compliant
* Merge branch 'master' of github.com:ros-drivers/axis_camera

0.3.2 (2021-05-21)
------------------
* Improve support for the F34 and F44 multi-camera controllers by adding default values for the camera index (1-4). Change the camera arg in view_axis to camera_name, change its default IP address to better-match with the main axis.launch file
* Contributors: Chris Iverach-Brereton

0.3.1 (2020-12-10)
------------------
* Merge pull request `#62 <https://github.com/ros-drivers/axis_camera/issues/62>`_ from ros-drivers/teleop-axis
  Fixed tele-op axis params.
* Merge pull request `#55 <https://github.com/ros-drivers/axis_camera/issues/55>`_ from sgemme-csa/master
  KeyError in publishCameraState when camera is not ready on PTZ camera
* Expose the height & width parameters as arguments in the launch file
* Merge pull request `#56 <https://github.com/ros-drivers/axis_camera/issues/56>`_ from jeff-o/patch-1
  Update axis.launch
* Merge pull request `#58 <https://github.com/ros-drivers/axis_camera/issues/58>`_ from luishowell/master
  add support for quad video
* Merge pull request `#61 <https://github.com/ros-drivers/axis_camera/issues/61>`_ from cclauss/patch-1
  Fix Python 3 syntax error
* Remove the html_static directory from conf.py; it doesn't exist anyway and is just creating a warning that's causing Jenkins to see the build as unstable
* Fix Python 3 syntax error
  `#52 <https://github.com/ros-drivers/axis_camera/issues/52>`_ again
* Remove the :: leftover from the .rst
* Copy the README contents to the .md so they show up on the github main page
* Update the maintainer now that Clearpath is officially maintaining this package again
* Merge pull request `#54 <https://github.com/ros-drivers/axis_camera/issues/54>`_ from k-okada/add_travis
* update travis.yml
* add support for quad video
* Update axis.launch
  Adds the "camera" param to the launch file. Helps launch the driver cleanly when used with other drivers that also use "camera" as a param name.
* No need to close connection as it will get garbage collected
* Merge remote-tracking branch 'csa/develop' into github-master
* Adjusting error message on KeyError
* Merge remote-tracking branch 'github/master' into develop
* Merge branch 'develop' of git+ssh://liberty/data/git/ros/axis_camera into develop
* Fixing camera telemetry where accessing its telemetry before a certain time after startup would causes a KeyError because the fields in the response were not present. Now catching the KeyError exception to fix the problem.
* Fixing camera telemetry where accessing its telemetr before a certain time after startup would cause a KeyError because the fiels in the response were not present, now catchin the KeyError exception to fix the problem
* Fixing connection problem which was causing the telemetry to stall
* Contributors: Chris I-B, Christian Clauss, Howell, Jeff Schmidt, Kei Okada, Sebastien Gemme, jmastrangelo-cpr

0.3.0 (2018-05-25)
------------------
* Merge pull request `#49 <https://github.com/ros-drivers/axis_camera/issues/49>`_ from rossctaylor/feature/support_for_f34
  Add: support for Axis F34 multicamera switch
* Merge pull request `#48 <https://github.com/ros-drivers/axis_camera/issues/48>`_ from tonybaltovski/pan-tilt-parms
  Added ROS params for the pan and tilt axis.
* Contributors: Ross Taylor, Tony Baltovski

0.2.1 (2017-11-17)
------------------
* add ros-orphaned-maintaner to package.xml (`#50 <https://github.com/ros-drivers/axis_camera/issues/50>`_)
* Set queue_size to Publishers in axis_camera (`#47 <https://github.com/ros-drivers/axis_camera/issues/47>`_)
* Point package.xml URLs at ros-drivers org. (`#39 <https://github.com/ros-drivers/axis_camera/issues/39>`_)
* sending camera_info (`#38 <https://github.com/ros-drivers/axis_camera/issues/38>`_)
  * copying stamp so rectification happens
  * sending camera_info
* Contributors: Kei Okada, Kentaro Wada, Mike Purvis, Sam Pfeiffer, Micah Corah

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
