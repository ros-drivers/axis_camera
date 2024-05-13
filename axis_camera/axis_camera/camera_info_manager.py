# Copyright 2024 Clearpath Robotics Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Clearpath Robotics Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


# Software License Agreement (BSD License)
#
# Copyright (C) 2012, Jack O'Quin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the author nor of other contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Python camera_info_manager interface, providing `CameraInfo` support
for drivers written in Python. This is very similar to the
`C++ camera_info_manager`_ package, but not identical.

.. _`C++ camera_info_manager`: http://ros.org/wiki/camera_info_manager
.. _`sensor_msgs/CameraInfo`: http://ros.org/doc/api/sensor_msgs/html/msg/CameraInfo.html
.. _`sensor_msgs/SetCameraInfo`: http://ros.org/doc/api/sensor_msgs/html/srv/SetCameraInfo.html

"""
# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import rospkg
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo
from sensor_msgs.srv import SetCameraInfoResponse

import os
import errno
import yaml

default_camera_info_url = "file://${ROS_HOME}/camera_info/${NAME}.yaml";

# parseURL() type codes:
URL_empty = 0                   # empty string
URL_file = 1                    # file:
URL_package = 2                 # package:
URL_invalid = 3                 # anything >= is invalid

class CameraInfoManager():
    """
    :class:`CameraInfoManager` provides ROS CameraInfo support for
    Python camera drivers. It handles the `sensor_msgs/SetCameraInfo`_
    service requests, saving and restoring `sensor_msgs/CameraInfo`_
    data.

    :param cname: camera name.
    :param url: Uniform Resource Locator for camera calibration data.
    :param namespace: Optional ROS namespace prefix for the service
           name.  If a namespace is specified, the '/' separator
           required between it and ``set_camera_info`` will be
           supplied automatically.

    .. describe:: str(camera_info_manager_obj)

       :returns: String representation of :class:`CameraInfoManager` object.

    **ROS Service**

    - set_camera_info (`sensor_msgs/SetCameraInfo`_) stores
                      calibration information

    Typically, these service requests are made by a calibration
    package, such as:

    - http://www.ros.org/wiki/camera_calibration

    The calling node *must* invoke `rospy.spin()` in some thread, so
    :class:`CameraInfoManager` gets called to handle arriving service
    requests.

    If a driver handles multiple cameras, it should use the
    ``namespace`` parameter to declare separate
    :class:`CameraInfoManager` instances for each one, as in this
    stereo example::

      left_ci = CameraInfoManager(cname='left_camera', namespace='left')
      right_ci = CameraInfoManager(cname='right_camera', namespace='right')

    **Camera Name**

    The device driver sets a camera name via the
    :class:`CameraInfoManager` constructor or the
    :py:meth:`setCameraName` method.  This name is written when
    CameraInfo is saved, and checked when data are loaded, with a
    warning logged if the name read does not match.

    Syntax: a camera name contains any combination of alphabetic,
            numeric and '_' characters.  Case is significant.

    Camera drivers may use any syntactically valid name they please.
    Where possible, it is best for the name to be unique to the
    device, such as a GUID, or the make, model and serial number.  Any
    parameters that affect calibration, such as resolution, focus,
    zoom, etc., may also be included in the name, uniquely identifying
    each CameraInfo file.

    The camera name can be resolved as part of the URL, allowing
    direct access to device-specific calibration information.

    **Uniform Resource Locator**

    The location for getting and saving calibration data is expressed
    by Uniform Resource Locator.  The driver defines a URL via the
    :class:`CameraInfoManager` constructor or the :py:meth:`setURL`
    method.  Many drivers provide a `~camera_info_url` parameter so
    users may customize this URL, but that is handled outside this
    class.

    Camera calibration information is stored in YAML format.

    Example URL syntax:

    - `file:///full/path/to/local/file.yaml`
    - `package://camera_info_manager_py/tests/test_calibration.yaml`
    - `package://ros_package_name/calibrations/camera3.yaml`

    The `file:` URL specifies a full path name in the local system.
    The `package:` URL is handled the same as `file:`, except the path
    name is resolved relative to the location of the named ROS
    package, which must be reachable via `$ROS_PACKAGE_PATH`.

    The URL may contain substitution variables delimited by `${...}`,
    including:

    - ${NAME} resolved to the current camera name defined by the
              device driver.
    - ${ROS_HOME} resolved to the `$ROS_HOME` environment variable if
                  defined, `~/.ros` if not.

    Resolution is done in a single pass through the URL string.
    Variable values containing substitutable strings are not resolved
    recursively.  Unrecognized variable names are treated literally
    with no substitution, but an error is logged.

    Examples with variable substitution:

    - `package://my_cameras/calibrations/${NAME}.yaml`
    - `file://${ROS_HOME}/camera_info/left_front_camera.yaml`

    The default URL is:

    - `file://${ROS_HOME}/camera_info/${NAME}.yaml`

    If that file exists, its contents are used. Any new calibration
    will be stored there, missing parent directories being created if
    necessary and possible.

    **Loading Calibration Data**

    Unlike the `C++ camera_info_manager`_, this Python implementation
    loads nothing until the :py:meth:`loadCameraInfo` method is
    called.  It is an error to call :py:meth:`getCameraInfo`, or
    :py:meth:`isCalibrated` before that is done.

    If the URL or camera name changes, :py:meth:`loadCameraInfo` must
    be called again before the data are accessible.

    """
    def __init__(self, cname='camera', url='', namespace=''):
        """Constructor.
        """
        self.cname = cname
        self.url = url
        self.camera_info = None

        # advertise set_camera_info service
        service_name = 'set_camera_info'
        if namespace:
            service_name = namespace + '/' + service_name
        #rospy.logdebug(service_name + ' service declared')
        self.svc = rospy.Service(service_name, SetCameraInfo,
                                 self.setCameraInfo)

    def __str__(self):
        """:returns: String representation of :class:`CameraInfoManager` """
        return '[' + self.cname + ']' + str(self.utm)

    def getCameraInfo(self):
        """ Get the current camera calibration.

        The :py:meth:`loadCameraInfo` must have been called since the
        last time the camera name or URL changed.

        :returns: `sensor_msgs/CameraInfo`_ message.

        :raises: :exc:`CameraInfoMissingError` if camera info not up
                 to date.

        """
        if self.camera_info is None:
            raise CameraInfoMissingError('Calibration missing, loadCameraInfo() needed.')
        return self.camera_info

    def getCameraName(self):
        """ Get the current camera name.

        :returns: camera name string
        """
        return self.cname

    def getURL(self):
        """ Get the current calibration URL.

        :returns: URL string without variable expansion.
        """
        return self.url

    def isCalibrated(self):
        """ Is the current CameraInfo calibrated?

        The :py:meth:`loadCameraInfo` must have been called since the
        last time the camera name or URL changed.

        :returns: True if camera calibration exists;
                  False for null calibration.

        :raises: :exc:`CameraInfoMissingError` if camera info not up
                 to date.

        """
        if self.camera_info is None:
            raise CameraInfoMissingError('Calibration missing, ' +
                                         'loadCameraInfo() needed.')
        return self.camera_info.K[0] != 0.0

    def _loadCalibration(self, url, cname):
        """ Load calibration data (if any available).

        This method updates self.camera_info, if possible, based on
        the url and cname parameters.  An empty or non-existent
        calibration is *not* considered an error, a null
        `sensor_msgs/CameraInfo`_ being provided in that case.

        :param url: Uniform Resource Locator for calibration data.
        :param cname: Camera name.

        :raises: :exc:`IOError` if an existing calibration is unreadable.
        :raises: :exc:`CameraInfoMissingError` if a `package:` URL is
                 inaccessible.
        """
        resolved_url = resolveURL(url, cname)
        url_type = parseURL(resolved_url)

        if url_type == URL_empty:
            self._loadCalibration(default_camera_info_url, cname)
            return

        rospy.loginfo('camera calibration URL: ' + resolved_url)

        if url_type == URL_file:
            self.camera_info = loadCalibrationFile(resolved_url[7:], cname)

        elif url_type == URL_package:
            filename = getPackageFileName(resolved_url)
            if filename == '':          # package not resolved
                raise CameraInfoMissingError('Calibration package missing.')
            self.camera_info = loadCalibrationFile(filename, cname)

        else:
            rospy.logerr("Invalid camera calibration URL: " + resolved_url)
            self.camera_info = CameraInfo()

    def loadCameraInfo(self):
        """ Load currently configured calibration data (if any).

        This method updates camera_info, if possible, based on the
        currently-configured URL and camera name.  An empty or
        non-existent calibration is *not* considered an error; a null
        `sensor_msgs/CameraInfo`_ being provided in that case.

        :raises: :exc:`IOError` if an existing calibration is unreadable.

        """
        self._loadCalibration(self.url, self.cname)

    def setCameraInfo(self, req):
        """ Callback for SetCameraInfo request.

        :param req: SetCameraInfo request message.
        :returns: SetCameraInfo response message, success is True if
                  message handled.

        :post: camera_info updated, can be used immediately without
               reloading.
        """
        rospy.logdebug('SetCameraInfo received for ' + self.cname)
        self.camera_info = req.camera_info
        rsp = SetCameraInfoResponse()
        rsp.success = saveCalibration(req.camera_info,
                                      self.url, self.cname)
        if not rsp.success:
            rsp.status_message = "Error storing camera calibration."
        return rsp

    def setCameraName(self, cname):
        """ Set a new camera name.

        :param cname: camera name to use for saving calibration data

        :returns: True if new name has valid syntax; valid names
                  contain only alphabetic, numeric, or '_' characters.

        :post: camera name updated, if valid. A new name may affect
               the URL, so camera_info will have to be reloaded before
               being used again.

        """
        # validate name
        if cname == '':
            return False        # name may not be empty
        for ch in cname:
            if not ch.isalnum() and ch != '_':
                return False    # invalid character

        # name is valid, use it
        if self.cname != cname:
            self.cname = cname
            self.camera_info = None     # missing if name changed
        return True

    def setURL(self, url):
        """ Set the calibration URL.

        :param cname: camera name to use for saving calibration data

        :returns: True if new name has valid syntax.

        :post: URL updated, if valid. A new value may change the
               camera_info, so it will have to be reloaded before
               being used again.

        """
        if parseURL(resolveURL(url, self.cname)) >= URL_invalid:
            return False                # syntax error

        # URL looks valid, so use it
        if self.url != url:
            self.url = url
            self.camera_info = None     # missing if URL changed
        return True

# related utility functions


def genCameraName(from_string):
    """ Generate a valid camera name.

    Valid names contain only alphabetic, numeric, or '_'
    characters. All invalid characters in from_string are replaced
    by an '_'.

    :param from_string: string from which to base camera name.

    :returns: a valid camera name based on from_string.

    """
    if from_string == '':
        return '_'          # name may not be empty

    retval = ''
    for i in range(len(from_string)):
        if not from_string[i].isalnum() and from_string[i] != '_':
            retval += '_'
        else:
            retval += from_string[i]
    return retval

