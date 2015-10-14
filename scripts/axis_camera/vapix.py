"""Python 2 implementation of the VAPIX HTTP API for communicating with Axis network cameras.

This class supports both VAPIX v2 and VAPIX v3, including autodetection of the supported API.
The VAPIX APIs are implemented based on these specifications:

- VAPIX v2: http://www.axis.com/files/manuals/HTTP_API_VAPIX_2.pdf
- VAPIX v3:

    - streaming: http://www.axis.com/files/manuals/vapix_video_streaming_52937_en_1307.pdf
    - PTZ control: http://www.axis.com/files/manuals/vapix_ptz_52933_en_1307.pdf

When an instance of the API is obtained, the capabilities and limits of the connected camera are read automatically
and they can be queried afterwards. A :py:exc:`exceptions.RuntimeError` will be thrown when calling an API method for which the camera
doesn't have capabilities.

:Example:

To get an autodetected API instance allowing you to access a connected camera, use the following code:

.. code-block:: python

    api = VAPIX.get_api_for_camera(hostname, username, password, camera_id, use_encrypted_password)

This call throws :py:exc:`IOError` or :py:exc:`ValueError` if connection to the camera did not succeed, so keep trying in
a loop until the camera becomes reachable.

.. note:: Tested with the following products. Tests of other products are welcome!

- Axis 214 PTZ (firmware 4.49) with VAPIX v2

.. note:: Always update your camera to the newest available firmware from
          http://origin-www.axis.com/ftp/pub_soft/cam_srv/ before issuing a bug.

"""

import urllib2
import socket
from contextlib import closing
from abc import ABCMeta, abstractmethod

# from rospy, we only use is_shutdown(), sleep() and logdebug/logwarn(), so if you need to use this
# API outside ROS, the transformation should be pretty easy (you could even provide a fake rospy package
# and then use this code unchanged)
import rospy


class Range(object):
    """A helper class representing an allowed parameter range."""

    def __init__(self, min=float('-inf'), max=float('inf'), period=None):
        """Create a parameter range.

        :param min: The minimum allowed value. Minus infinity means no lower limit.
        :type min: float, int
        :param max: The maximum allowed value. Plus infinity means no upper limit.
        :type max: float, int
        :param period: If the value is periodic (e.g. angles), specify the size of the period and all values
                       passed to this range will first be "normalized" to the range -period/2 to period/2.
        :type period: float, int
        """
        self.min = min
        self.max = max
        self.period = period

    def _normalize(self, value):
        """Adjust the value to fit inside the period if this is a periodic range. Has no effect on a-periodic ranges.

        :param value: The value to normalize.
        :type value: float, int
        :return: The normalized value.
        :rtype: :py:class:`float`, :py:class:`int`
        """
        if self.period is None:
            return value

        result = value
        while result > self.period/2.0:
            result -= self.period
        while result < -self.period / 2.0:
            result += self.period
        return result

    def is_in_range(self, value):
        """Check if the given value satisfies this range's constraints (after being normalized).

        :param value: The value to check.
        :type value: float, int
        :return: If the value fits this range.
        :rtype: :py:class:`bool`
        """
        return self.min <= self._normalize(value) <= self.max

    def crop_value(self, value):
        """If the value is outside this range, return the corresponding limit, otherwise just return the value.

        :param value: The value to crop.
        :type value: float, int
        :return: The cropped and normalized value.
        :rtype: :py:class:`float`, :py:class:`int`
        """
        value = self._normalize(value)

        if value > self.max:
            return self.max
        elif value < self.min:
            return self.min
        else:
            return value

    def merge(self, range):
        """Merge this range with another one (using logical AND to join the conditions).
        If both ranges share the same period, it is retained. If only one of the ranges is periodic, its
        period is used for the resulting range. If both ranges are periodic with different period, a :class:ValueError
        is thrown.

        :param range: The range to merge with.
        :type range: :py:class:`Range`
        :return: A new range satisfying the limits of both this and the given range.
        :rtype: :py:class:`Range`
        :raises: :py:exc:`ValueError` if both ranges are periodic, and their periods differ
        """
        assert isinstance(range, Range)

        period = None
        if self.period is not None:
            if range.period is not None:
                if self.period != range.period:
                    raise ValueError("Cannot join two ranges with different periods.")
                period = self.period
            else:
                period = self.period
        elif range.period is not None:
                period = range.period

        return Range(max(range.min, self.min), min(range.max, self.max), period)

    def __str__(self):
        result = "<%f, %f>" % (self.min, self.max)
        if self.period is not None:
            result += " + k*%f" % self.period
        return result

    def __repr__(self):
        return "Range(min=%r, max=%r, period=%r)" % (self.min, self.max, self.period)


class PTZLimit(object):
    """A helper class holding information about absolute, relative and velocity parameter ranges.

    This is useful for representing the pan, tilt or zoom parameter limits.
    """

    def __init__(self, absolute=None, relative=None, velocity=None):
        """Create a limit for absolute, relative and velocity parameters.

        :param absolute: The allowed range for absolute movement.
        :type absolute: :py:class:`Range`
        :param relative: The allowed range for relative movement.
        :type relative: :py:class:`Range`
        :param velocity: The allowed range for velocity.
        :type velocity: :py:class:`Range`
        """
        self.absolute = absolute if absolute is not None else Range()
        self.relative = relative if relative is not None else Range()
        self.velocity = velocity if velocity is not None else Range()

    def merge(self, limit):
        """Merge this limit with another one.

        :param limit: The limit to merge with.
        :type limit: :py:class:`PTZLimit`
        :return: A new limit satisfying the conditions of this and the given one.
        :rtype: :py:class:`PTZLimit`
        """
        assert isinstance(limit, PTZLimit)

        return PTZLimit(
            self.absolute.merge(limit.absolute),
            self.relative.merge(limit.relative),
            self.velocity.merge(limit.velocity)
        )

    def __str__(self):
        return "[absolute=%s, relative=%s, velocity=%s]" % (str(self.absolute), str(self.relative), str(self.velocity))

    def __repr__(self):
        return "PTZLimit(absolute=%r, relative=%r, velocity=%r)" % (self.absolute, self.relative, self.velocity)


class VAPIX(object):
    """Implementation of the VAPIX HTTP API of Axis network cameras.

    This is an abstract common ancestor for both API v2 and v3, since they share the most of the functionality.
    The differences between v2 and v3 are specified by overriding this class' methods in a subclass.
    """
    __metaclass__ = ABCMeta

    def __init__(self, hostname, camera_id=1, connection_timeout=5):
        """An instance of the VAPIX API.

        :param hostname: Hostname or IP address string of the connected camera.
        :type hostname: basestring
        :param camera_id: Id of the camera to connect to (if more video sources are available). Usually a number 1-4.
        :type camera_id: int
        :param connection_timeout: Timeout of API requests (in seconds).
        :type connection_timeout: int
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on connection error
        """
        self.hostname = hostname
        self.camera_id = camera_id
        self.connection_timeout = connection_timeout

        self._has_ptz = self.has_ptz()
        if self._has_ptz:
            self._ptz_capabilities = self._get_ptz_capabilities()
            self.ptz_limits = self._get_ptz_limits()

    @abstractmethod
    def _form_parameter_url(self, group):
        """Construct a URL for querying the specified parameter group.

        :param group: The parameter group to get using the generated URL. Can be both a group and a single parameter.
        :type group: basestring
        :return: The full API URL to query.
        :rtype: :py:obj:`basestring`
        """
        pass

    def get_video_stream(self, fps=24, resolution_name='CIF', compression=0, use_color=True, use_square_pixels=False):
        """Return a stream connected to the camera's MJPG video source.

        :param fps: The desired frames per second.
        :type fps: int
        :param resolution_name: The CIF standard name of the requested stream resolution (e.g. 4CIF, 2CIFEXP, CIF).
        :type resolution_name: basestring
        :param compression: Compression of the image (0 - no compression, 100 - max compression).
        :type compression: int
        :param use_color: If True, send a color stream, otherwise send only grayscale image.
        :type use_color: bool
        :param use_square_pixels: If True, the resolution will be stretched to match 1:1 pixels.
                                  By default, the pixels have a ratio of 11:12.
        :return: A handle to the open stream.
        :rtype: :py:class:`urllib.addinfourl` (a :py:obj:`file`-like object)
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError`

        .. note:: The data on the stream are not directly an MJPEG stream. They need to be parsed according to VAPIX.
        """
        api_call = 'axis-cgi/mjpg/video.cgi?camera=%d&fps=%d&resolution=%s&compression=%d&color=%d&squarepixel=%d' % (
            self.camera_id, fps, resolution_name, compression, 1 if use_color else 0, 1 if use_square_pixels else 0
        )

        url = self._form_api_url(api_call)
        return self._open_url(url, valid_statuses=[200])

    def is_video_ok(self):
        """Check using API whether the video source is ready and streaming.

        :return: If the video source is OK.
        :rtype: :py:class:`bool`
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError`

        .. note:: Unfortunately, not all video errors are recognized (e.g. the black-image stream).
        """
        url = self._form_api_url("axis-cgi/view/videostatus.cgi?status=%d" % self.camera_id)
        response_line = self._read_oneline_response(url)
        status = self.parse_parameter_and_value_from_response_line(response_line)[1]

        return status == "video"

    @staticmethod
    def read_next_image_from_video_stream(stream):
        """Read the next image header and image from the stream, skipping any irrelevant data in the stream.

        The stream is supposed to be in a state such that an image and the empty line after it have just been read (or
        at the very start of the stream).

        :param stream: The video stream.
        :type stream: :py:class:`urllib.addinfourl`
        :return: Video frame header and the frame. Return (None, None) if reading the image fails. The frame is in JPG.
        :rtype: tuple (dict, :py:obj:`basestring`)
        :raises: :py:exc:`IOError`, :py:exc:`urllib.URLError` If reading from the stream fails.
        """
        found_boundary = VAPIX._find_boundary_in_stream(stream)
        if not found_boundary:
            return None, None

        return VAPIX._read_image_from_stream(stream)

    @staticmethod
    def _find_boundary_in_stream(stream):
        """The string "--myboundary" is used to denote the start of an image in Axis cameras.

        :param stream: The video stream.
        :type stream: urllib.addinfourl
        :return: Returns False if end of stream was reached.
        :rtype: :py:class:`bool`
        :raises: :py:exc:`IOError`, :py:exc:`urllib.URLError` If reading from the stream fails.
        """
        while not rospy.is_shutdown():
            line = stream.readline()
            if line is None:
                # end of stream
                return False
            if line == '--myboundary\r\n':
                return True
                # throw away all other data until a boundary is found

    @staticmethod
    def _read_image_from_stream(stream):
        """Get the image header and image itself.

        The stream is supposed to be in a state such that the boundary is the last line read from the stream.

        :param stream: The video stream.
        :type stream: urllib.addinfourl
        :return: Video header and the frame. Return (None, None) if reading the image fails. The frame is in JPG.
        :rtype: tuple (dict, :py:obj:`basestring`)
        :raises: :py:exc:`IOError`, :py:exc:`urllib.URLError` If reading from the stream fails.
        """
        header = VAPIX._get_image_header_from_stream(stream)

        image = None
        if header['Content-Length'] > 0:
            content_length = int(header['Content-Length'])
            image = VAPIX._get_image_data_from_stream(stream, content_length)
        else:
            return None, None

        return header, image

    @staticmethod
    def _get_image_header_from_stream(stream):
        """Read an HTTP-like header of the next video stream.

        The stream is supposed to be in a state such that the boundary is the last line read from the stream.

        :param stream: The video stream.
        :type stream: urllib.addinfourl
        :return: A dcitionary of HTTP-like headers. Most notably, the 'Content-Length' denotes the byte-size of the next
                 video frame, which follows right after this header in the stream.
        :rtype: dict
        :raises: :py:exc:`IOError`, :py:exc:`urllib.URLError` If reading from the stream fails.
        """
        header = {}
        while not rospy.is_shutdown():
            line = stream.readline()
            if line == "\r\n":  # the header is finished with an empty line
                break
            line = line.strip()
            parts = line.split(": ", 1)

            if len(parts) != 2:
                rospy.logwarn('Problem encountered with image header. Setting content_length to zero. The problem '
                              'header was: "%s"' % line)
                header['Content-Length'] = 0  # set content_length to zero if there is a problem reading header
            else:
                try:
                    header[parts[0]] = parts[1]
                except KeyError, e:
                    rospy.logwarn('Problem encountered with image header. Invalid header key: %r' % e)
                    header['Content-Length'] = 0  # set content_length to zero if there is a problem reading header

        return header

    @staticmethod
    def _get_image_data_from_stream(stream, num_bytes):
        """Get the binary image data itself (ie. without header)

        The stream is supposed to be in a state such that the empty line after header is the last line read from the
        stream.

        :param stream: The video stream.
        :type stream: urllib.addinfourl
        :param num_bytes: The byte-size of the video frame.
        :type num_bytes: int
        :return: The byte data of the video frame (encoded as JPG).
        :rtype: :py:class:`str`
        :raises: :py:exc:`IOError`, :py:exc:`urllib.URLError` If reading from the stream fails.
        """
        image = stream.read(num_bytes)
        stream.readline()  # Read terminating \r\n and do nothing with it
        return image

    def get_camera_position(self, get_zoom=True, get_focus=True, get_iris=True):
        """Get current camera PTZ position.

        :param get_zoom: If true, also try to get zoom. Set to false if using with no-zoom cameras.
        :type get_zoom: bool
        :param get_focus: If true, also try to get autofocus and focus. Set to false if using with no-focus cameras.
                          Value True is deprecated.
        :type get_focus: bool
        :param get_iris: If true, also try to get autoiris and iris. Set to false if using with no-iris cameras.
                         Value True is deprecated.
        :type get_iris: bool
        :return: The current camera position.
        :rtype: dict {'pan': ..., 'tilt': ..., ['zoom': ...]}
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError`

        .. note:: The get_focus and get_iris parameters are only supported for backwards compatibility.
        """
        url = self._form_api_url("axis-cgi/com/ptz.cgi?query=position&camera=%d" % self.camera_id)
        response_lines = self._read_multiline_response(url)

        position_keys = {'pan', 'tilt'}
        if get_zoom:
            position_keys.add('zoom')
        if get_focus:  # deprecated
            position_keys.add('autofocus')
            position_keys.add('focus')
        if get_iris:  # deprecated
            position_keys.add('autoiris')
            position_keys.add('iris')

        # now read everything returned in the API response and pick up the data we are requested for
        position = dict()
        for line in response_lines:
            (key, value) = self.parse_parameter_and_value_from_response_line(line)
            if key in position_keys:
                if not key.startswith('auto'):
                    position[key] = float(value)
                else:
                    position[key] = True if value == 'on' else False

        if 'pan' not in position or 'tilt' not in position or (get_zoom and 'zoom' not in position):
            rospy.logdebug('Only succeeded to parse the following position values: %r' % position)
            raise RuntimeError('Error requesting the current position of the camera. Unexpected API response.')

        return position

    def take_snapshot(self):
        """Take a snapshot at the current camera's position.

        :return: The binary data of the image.
        :rtype: :py:obj:`bytes`
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError`

        .. todo:: Implement more snapshotting functionality.
        """
        image_data = self._read_binary_response(self._form_api_url("axis-cgi/jpg/image.cgi?camera=%d" % self.camera_id))
        return image_data

    def restart_camera(self):
        """Restart (re-initialize) the camera.

        This requires admin credentials to be given when creating this API instance.
        """
        rospy.loginfo("Restarting camera %d on %s ." % (self.camera_id, self.hostname))
        self._call_api_no_response("axis-cgi/admin/restart.cgi")

        # and now try to reload camera capabilities which might have changed after restart
        # (here we also just wait until the camera is ready again)
        while not rospy.is_shutdown():
            try:
                self._has_ptz = self.has_ptz()
                if self._has_ptz:
                    self._ptz_capabilities = self._get_ptz_capabilities()
                    self.ptz_limits = self._get_ptz_limits()
                break
            except (IOError, ValueError, RuntimeError):
                rospy.logdebug("Cannot get camera capabilities. Waiting 1s.")
                rospy.sleep(1)

    ################
    # PTZ movement #
    ################

    def move_ptz_absolute(self, pan=None, tilt=None, zoom=None):
        """Command the PTZ unit with an absolute pose.

        :param pan: The desired pan. In None, pan is not commanded at all.
                    The value is automatically normalized to <-180,+180>
        :type pan: float
        :param tilt: The desired tilt. In None, tilt is not commanded at all.
                     The value is automatically normalized to <-180,+180>
        :type tilt: float
        :param zoom: The desired zoom level. In None, zoom is not commanded at all.
        :type zoom: int
        :return: The pan, tilt and zoom values that were really applied (e.g. the cropped and normalized input)
        :rtype: tuple

        :raises: :py:exc:`RuntimeError` if the camera doesn't have capabilities to execute the given command.
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error

        .. note:: This call doesn't wait for the command to finish execution (which might take a while if e.g. a move
                  from -170 to +170 pan is requested).

        .. note:: Since the pan and tilt values are periodic and normalization takes place in this function, you can
                  simply call this command in loops like e.g. `for pan in range(0,3600,30): move_ptz_absolute(pan)`
                  to rotate the camera 10 times.
        """
        commands = []
        if pan is not None:
            self.require_capabilities('AbsolutePan')
            pan = self.ptz_limits['Pan'].absolute.crop_value(pan)
            commands.append("pan=%f" % pan)

        if tilt is not None:
            self.require_capabilities('AbsoluteTilt')
            tilt = self.ptz_limits['Tilt'].absolute.crop_value(tilt)
            commands.append("tilt=%f" % tilt)

        if zoom is not None:
            self.require_capabilities('AbsoluteZoom')
            zoom = self.ptz_limits['Zoom'].absolute.crop_value(zoom)
            commands.append("zoom=%d" % zoom)

        if len(commands) > 0:
            command = "&".join(commands)
            self._call_ptz_command(command)

        return pan, tilt, zoom

    def move_ptz_relative(self, pan=None, tilt=None, zoom=None):
        """Command the PTZ unit with a relative pose shift.

        :param pan: The pan change. In None or 0, pan remains unchanged.
                    The value is automatically normalized to <-360,+360>. May be negative.
        :type pan: float
        :param tilt: The tilt change. In None or 0, tilt remains unchanged.
                     The value is automatically normalized to <-360,+360>. May be negative.
        :type tilt: float
        :param zoom: The zoom change. In None or 0, zoom remains unchanged. May be negative.
        :type zoom: int
        :return: The pan, tilt and zoom change values that were really applied (e.g. the cropped and normalized input)
        :rtype: tuple

        :raises: :py:exc:`RuntimeError` if the camera doesn't have capabilities to execute the given command.
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error

        .. note:: This call doesn't wait for the command to finish execution (which might take a while if e.g. a move
                  by 300 deg pan is requested).
        """
        commands = []

        if pan is not None:
            self.require_capabilities('RelativePan')
            pan = self.ptz_limits['Pan'].relative.crop_value(pan)
            commands.append("rpan=%f" % pan)

        if tilt is not None:
            self.require_capabilities('RelativeTilt')
            tilt = self.ptz_limits['Tilt'].relative.crop_value(tilt)
            commands.append("rtilt=%f" % tilt)

        if zoom is not None:
            self.require_capabilities('RelativeZoom')
            zoom = self.ptz_limits['Zoom'].relative.crop_value(zoom)
            commands.append("rzoom=%d" % zoom)

        if len(commands) > 0:
            command = "&".join(commands)
            self._call_ptz_command(command)

        return pan, tilt, zoom

    def set_ptz_velocity(self, pan=None, tilt=None, zoom=None):
        """Command the PTZ unit velocity in terms of pan, tilt and zoom.

        :param pan: Pan speed. In None or 0, pan remains unchanged. Pan speed is aperiodic (can be higher than 360).
                    May be negative.
        :type pan: int
        :param tilt: Tilt speed. In None or 0, tilt remains unchanged. Tilt speed is aperiodic (can be higher than 360).
                    May be negative.
        :type tilt: int
        :param zoom: Zoom speed. In None or 0, zoom remains unchanged. May be negative.
        :type zoom: int
        :return: The pan, tilt and zoom velocity values that were really applied (e.g. the cropped and normalized input)
        :rtype: tuple

        :raises: :py:exc:`RuntimeError` if the camera doesn't have capabilities to execute the given command.
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error

        .. note:: This call doesn't wait for the command to finish execution.

        .. note:: It is not clearly stated in the API, but practically any following absolute/relative pose command
                  sets velocity to zero.
        """
        commands = []

        if pan is not None:
            self.require_capabilities('ContinuousPan')
            pan = self.ptz_limits['Pan'].velocity.crop_value(pan)
        else:
            pan = 0

        if tilt is not None:
            self.require_capabilities('ContinuousTilt')
            tilt = self.ptz_limits['Tilt'].velocity.crop_value(tilt)
        else:
            tilt = 0

        if pan != 0 or tilt != 0:
            commands.append("continuouspantiltmove=%d,%d" % (pan, tilt))

        if zoom is not None:
            self.require_capabilities('ContinuousZoom')
            zoom = self.ptz_limits['Zoom'].velocity.crop_value(zoom)
            commands.append("continuouszoommove=%d" % zoom)

        if len(commands) > 0:
            command = "&".join(commands)
            self._call_ptz_command(command)

        return pan, tilt, zoom

    def look_at(self, x, y, image_width, image_height):
        """Point the camera center to a point with the given coordinates in the camera image.

        :param x: X coordinate of the look-at point.
        :type x: int
        :param y: X coordinate of the look-at point.
        :type y: int
        :param image_width: Width of the image in pixels.
        :type image_width: int
        :param image_height: Height of the image in pixels.
        :type image_height: int

        :raises: :py:exc:`RuntimeError` if the camera doesn't have capabilities to execute the given command.
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error

        .. todo:: A workaround for cameras without this functionality but with relative zoom support.
        """
        # these capabilities do not in fact ensure the capability needed for the center command, but better than nothing
        self.require_capabilities('AbsolutePan', 'AbsoluteTilt', 'AbsoluteZoom')

        self._call_ptz_command("center=%d,%d&imagewidth=%d&imageheight=%d" % (x, y, image_width, image_height))

    #########
    # Focus #
    #########

    def use_autofocus(self, use):
        """Command the camera to use/stop using autofocus.

        :param use: True: use autofocus; False: do not use it.
        :type use: bool
        :return: use
        :rtype: :py:class:`bool`

        :raises: :py:exc:`RuntimeError` if the camera doesn't have capabilities to execute the given command.
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error
        """
        self.require_capabilities('AutoFocus')
        self._call_ptz_command("autofocus=%s" % "on" if use else "off")
        return use

    def set_focus(self, focus):
        """Set focus to the desired value (implies turning off autofocus).

        :param focus: The desired focus value.
        :type focus: int
        :return: The focus value that was really applied (e.g. the cropped and normalized input)
        :rtype: :py:class:`int`

        :raises: :py:exc:`RuntimeError` if the camera doesn't have capabilities to execute the given command.
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error
        """
        self.require_capabilities('AbsoluteFocus')
        focus = self.ptz_limits['Focus'].absolute.crop_value(focus)
        self._call_ptz_command("autofocus=off&focus=%d" % focus)
        return focus

    def adjust_focus(self, amount):
        """Add the desired amount to the focus value (implies turning off autofocus).

        :param amount: The desired focus change amount.
        :type amount: int
        :return: The focus change amount that was really applied (e.g. the cropped and normalized input)
        :rtype: :py:class:`int`

        :raises: :py:exc:`RuntimeError` if the camera doesn't have capabilities to execute the given command.
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error
        """
        self.require_capabilities('RelativeFocus')
        amount = self.ptz_limits['Focus'].relative.crop_value(amount)
        self._call_ptz_command("autofocus=off&rfocus=%d" % amount)
        return amount

    def set_focus_velocity(self, velocity):
        """Set the focus "speed" (implies turning off autofocus).

        :param velocity: The desired focus velocity.
        :type velocity: int
        :return: The focus velocity that was really applied (e.g. the cropped and normalized input)
        :rtype: :py:class:`int`

        :raises: :py:exc:`RuntimeError` if the camera doesn't have capabilities to execute the given command.
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error
        """
        self.require_capabilities('ContinuousFocus')
        velocity = self.ptz_limits['Focus'].velocity.crop_value(velocity)
        self._call_ptz_command("autofocus=off&countinuousfocusmove=%d" % velocity)
        return velocity

    # Iris
    def use_autoiris(self, use):
        """Command the camera to use/stop using auto iris adjustment.

        :param use: True: use auto iris adjustment; False: do not use it.
        :type use: bool
        :return: use
        :rtype: :py:class:`bool`

        :raises: :py:exc:`RuntimeError` if the camera doesn't have capabilities to execute the given command.
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error
        """
        self.require_capabilities('AutoIris')
        self._call_ptz_command("autoiris=%s" % "on" if use else "off")
        return use

    def set_iris(self, iris):
        """Set iris to the desired value (implies turning off autoiris).

        :param iris: The desired iris value.
        :type iris: int
        :return: The iris value that was really applied (e.g. the cropped and normalized input)
        :rtype: :py:class:`int`

        :raises: :py:exc:`RuntimeError` if the camera doesn't have capabilities to execute the given command.
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error
        """
        self.require_capabilities('AbsoluteIris')
        iris = self.ptz_limits['Iris'].absolute.crop_value(iris)
        self._call_ptz_command("autoiris=off&iris=%d" % iris)
        return iris

    def adjust_iris(self, amount):
        """Add the desired amount to the iris value (implies turning off autoiris).

        :param amount: The desired iris change amount.
        :type amount: int
        :return: The iris change amount that was really applied (e.g. the cropped and normalized input)
        :rtype: :py:class:`int`

        :raises: :py:exc:`RuntimeError` if the camera doesn't have capabilities to execute the given command.
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error
        """
        self.require_capabilities('RelativeIris')
        amount = self.ptz_limits['Iris'].relative.crop_value(amount)
        self._call_ptz_command("autoiris=off&riris=%d" % amount)
        return amount

    def set_iris_velocity(self, velocity):
        """Set the iris "speed" (implies turning off autoiris).

        :param velocity: The desired iris velocity.
        :type velocity: int
        :return: The iris velocity that was really applied (e.g. the cropped and normalized input)
        :rtype: :py:class:`int`

        :raises: :py:exc:`RuntimeError` if the camera doesn't have capabilities to execute the given command.
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error
        """
        self.require_capabilities('ContinuousIris')
        velocity = self.ptz_limits['Iris'].velocity.crop_value(velocity)
        self._call_ptz_command("autoiris=off&countinuousirismove=%d" % velocity)
        return velocity

    # Brightness
    def set_brightness(self, brightness):
        """Set brightness to the desired value.

        :param brightness: The desired brightness value.
        :type brightness: int
        :return: The brightness value that was really applied (e.g. the cropped and normalized input)
        :rtype: :py:class:`int`

        :raises: :py:exc:`RuntimeError` if the camera doesn't have capabilities to execute the given command.
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error

        .. note:: The brightness setting has no effect on Axis 214 PTZ.
        """
        # self.require_capabilities('AbsoluteBrightness')  # this capability is not present in the list
        brightness = self.ptz_limits['Brightness'].absolute.crop_value(brightness)
        self._call_ptz_command("brightness=%d" % brightness)
        return brightness

    def adjust_brightness(self, amount):
        """Add the desired amount to the brightness value.

        :param amount: The desired brightness change amount.
        :type amount: int
        :return: The brightness change amount that was really applied (e.g. the cropped and normalized input)
        :rtype: :py:class:`int`

        :raises: :py:exc:`RuntimeError` if the camera doesn't have capabilities to execute the given command.
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error

        .. note:: The brightness setting has no effect on Axis 214 PTZ.
        """
        self.require_capabilities('RelativeBrightness')
        amount = self.ptz_limits['Brightness'].relative.crop_value(amount)
        self._call_ptz_command("rbrightness=%d" % amount)
        return amount

    def set_brightness_velocity(self, velocity):
        """Set the brightness "speed".

        :param velocity: The desired brightness velocity.
        :type velocity: int
        :return: The brightness velocity that was really applied (e.g. the cropped and normalized input)
        :rtype: :py:class:`int`

        :raises: :py:exc:`RuntimeError` if the camera doesn't have capabilities to execute the given command.
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error

        .. note:: The brightness setting has no effect on Axis 214 PTZ.
        """
        self.require_capabilities('ContinuousBrightness')
        velocity = self.ptz_limits['Brightness'].velocity.crop_value(velocity)
        self._call_ptz_command("countinuousbrightnessmove=%d" % velocity)
        return velocity

    ##########################
    # Backlight compensation #
    ##########################

    def use_backlight_compensation(self, use):
        """Command the camera to use/stop using backlight compensation (requires autoiris=on set before).

        :param use: True: use backlight compensation; False: do not use it.
        :type use: bool
        :return: use
        :rtype: :py:class:`bool`

        :raises: :py:exc:`RuntimeError` if the camera doesn't have capabilities to execute the given command.
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error
        """
        self.require_capabilities('BackLight')
        self._call_ptz_command("backlight=%s" % ("on" if use else "off"))
        return use

    ###################
    # Infrared filter #
    ###################

    def use_ir_cut_filter(self, use):
        """Command the camera to use/stop using infrared filter.

        :param use: None: Automatic mode (requires autoiris=on set before);
                    True: use infrared filter;
                    False: do not use it.
        :type use: bool
        :return: use
        :rtype: :py:class:`bool`

        :raises: :py:exc:`RuntimeError` if the camera doesn't have capabilities to execute the given command.
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error
        """
        self.require_capabilities('IrCutFilter')
        self._call_ptz_command("ircutfilter=%s" % ("auto" if use is None else ("on" if use else "off")))
        return use

    #######################
    # PTZ presence checks #
    #######################

    def has_ptz(self):
        """Returns True if a PTZ unit is available for the connected camera (according to the API).

        :return: Whether PTZ is available.
        :rtype: :py:class:`bool`
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error
        """
        return self.get_parameter("root.Properties.PTZ.PTZ") == "yes"

    def is_digital_ptz(self):
        """Returns True if a the PTZ unit is digital (i.e. has no mechanical parts).

        :return: Whether PTZ is digital or not.
        :rtype: :py:class:`bool`
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error
        """
        return self.get_parameter("root.Properties.PTZ.DigitalPTZ") == "yes"

    ###############################
    # Camera and PTZ capabilities #
    ###############################

    def has_capability(self, capability):
        """Tell whether the camera has the given capability.

        The list of capabilities is only queried once when connecting to the API, and calls to this method use the
        cached list of detected capabilities.

        :param capability: Name of the capability to test. E.g. 'AbsolutePan', 'ContinuousZoom' and so on (as defined
                           in VAPIX parameter group root.PTZ.Support)
        :type capability: basestring
        :return: Whether the camera has the given capability or not.
        :rtype: :py:class:`bool`
        """
        if not self._has_ptz:
            return False
        return capability in self._ptz_capabilities

    def has_capabilities(self, *capabilities):
        """Tell whether the camera has all of the given capabilities.

        The list of capabilities is only queried once when connecting to the API, and calls to this method use the
        cached list of detected capabilities.

        :param capabilities: Names of the capabilities to test. E.g. 'AbsolutePan', 'ContinuousZoom' and so on
                            (as defined in VAPIX parameter group root.PTZ.Support)
        :type capabilities: list of :py:obj:`basestring`
        :return: Whether the camera has all the given capabilities or not.
        :rtype: :py:class:`bool`
        """
        if not self._has_ptz:
            return False

        for capability in capabilities:
            if not self.has_capability(capability):
                return False

        return True

    def require_capabilities(self, *capabilities):
        """Raise a :py:exc:`RuntimeError` if the camera doesn't have all of the given capabilities.

        The list of capabilities is only queried once when connecting to the API, and calls to this method use the
        cached list of detected capabilities.

        :param capabilities: Names of the capabilities to require. E.g. 'AbsolutePan', 'ContinuousZoom' and so on
                            (as defined in VAPIX parameter group root.PTZ.Support)
        :type capabilities: list of :py:obj:`basestring`
        :raises: :py:exc:`RuntimeError` if the camery doesn't have one of the required capabilities.
        """
        for capability in capabilities:
            if not self.has_capability(capability):
                raise RuntimeError("Camera %d on host %s doesn't support the required capability %s." % (
                    self.camera_id, self.hostname, capability))

    def _get_ptz_capabilities(self):
        """Read the list of capabilities from the camera.

        :return: The list of capabilities.
        :rtype: list
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error
        """
        prefix = "root.PTZ.Support.S%d" % self.camera_id
        parameters = self.get_parameter_list(prefix)

        result = set()
        for (key, value) in parameters.iteritems():
            if value == "true":
                key_stripped = key.replace(prefix + ".", "")
                result.add(key_stripped)

        return result

    ##############
    # PTZ limits #
    ##############

    def _get_ptz_limits(self):
        """Return the list of effective limits of the PTZ control parameters.

        Effective means it is a combination of the VAPIX specifications and real limits read from the camera.

        :return: The limits of the PTZ control of the connected camera. Keys are 'Pan', 'Tilt' and so on.
        :rtype: dict (:py:obj:`basestring` => :py:class:`PTZLimit`)
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error
        """
        real_limits = self._get_real_ptz_limits()
        api_limits = self._get_api_ptz_limits()

        result = dict()
        # get the set of keys present in at least one of the two limit dicts
        keys = set().union(real_limits.keys()).union(api_limits.keys())

        # create the result so that if a limit is only in one dict, then just copy it, and when it is in both dicts,
        # put a merged limit in the result
        for key in keys:
            if key not in real_limits.keys():
                if key in api_limits.keys():
                    result[key] = api_limits[key]
            elif key not in api_limits.keys():
                if key in real_limits.keys():
                    result[key] = real_limits[key]
            else:
                result[key] = api_limits[key].merge(real_limits[key])

        return result

    def _get_real_ptz_limits(self):
        """Get the list of actual PTZ limits read from the camera.

        :return: The limits of the PTZ control of the connected camera. Keys are 'Pan', 'Tilt' and so on.
        :rtype: dict (:py:obj:`basestring` => :py:class:`PTZLimit`)
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error
        """
        prefix = "root.PTZ.Limit.L%d" % self.camera_id
        parameters = self.get_parameter_list(prefix)

        result = dict()
        for (key, value) in parameters.iteritems():
            key_stripped = key.replace(prefix + ".", "")

            limit = float(value)
            capability = key_stripped.replace("Min", "").replace("Max", "")

            if capability not in result.keys():
                result[capability] = PTZLimit()

            if key_stripped.startswith("Min"):
                result[capability].absolute.min = limit
            else:
                result[capability].absolute.max = limit

        result['Pan'].absolute.period = 360
        result['Tilt'].absolute.period = 360

        return result

    @staticmethod
    def _get_api_ptz_limits():
        """Get the list of PTZ limits specified by VAPIX.

        :return: The PTZ control limits. Keys are 'Pan', 'Tilt' and so on.
        :rtype: dict (:py:obj:`basestring` => :py:class:`PTZLimit`)
        """
        return {
            'Pan': PTZLimit(absolute=Range(-180, 180, 360), relative=Range(-360, 360, 720), velocity=Range(-100, 100)),
            'Tilt': PTZLimit(absolute=Range(-180, 180, 360), relative=Range(-360, 360, 720), velocity=Range(-100, 100)),
            'Zoom': PTZLimit(absolute=Range(1, 9999), relative=Range(-9999, 9999), velocity=Range(-100, 100)),
            'Focus': PTZLimit(absolute=Range(1, 9999), relative=Range(-9999, 9999), velocity=Range(-100, 100)),
            'Iris': PTZLimit(absolute=Range(1, 9999), relative=Range(-9999, 9999), velocity=Range(-100, 100)),
            'Brightness': PTZLimit(absolute=Range(1, 9999), relative=Range(-9999, 9999), velocity=Range(-100, 100)),
        }

    ######################
    # Parameter handling #
    ######################

    def get_parameter(self, name):
        """Get the value of the specified parameter through VAPIX.

        :param name: Full name of the parameter (e.g. 'root.Properties.PTZ.PTZ').
        :type name: basestring
        :return: The value of the parameter.
        :rtype: :py:obj:`basestring`
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error
        :raises: :py:exc:`ValueError` if response to the parameter request cannot be parsed
        """
        url = self._form_parameter_url(name)
        response_line = self._read_oneline_response(url)
        value = self.parse_parameter_and_value_from_response_line(response_line)
        return value[1]

    def get_parameter_list(self, group):
        """Get the values of all parameters under the specified parameter group through VAPIX.

        :param group: Full name of the group (e.g. 'root.Properties').
        :type group: basestring
        :return: The values of the parameters.
        :rtype: dict(:py:obj:`basestring` => :py:obj:`basestring`)
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network communication error
        :raises: :py:exc:`ValueError` if response to the parameter request cannot be parsed
        """
        url = self._form_parameter_url(group)
        response_lines = self._read_multiline_response(url)

        result = dict()
        for response_line in response_lines:
            parameter_and_value = self.parse_parameter_and_value_from_response_line(response_line)
            result[parameter_and_value[0]] = parameter_and_value[1]

        return result

    ########################
    # API HELPER FUNCTIONS #
    ########################

    def _form_api_url(self, api_call):
        """Return the URL to be called to execute the given API call.

        :param api_call: The API call without hostname and slash, e.g. "axis-cgi/param.cgi?camera=1".
        :type api_call: basestring
        :return: The URL.
        :rtype: :py:obj:`basestring`
        """
        return "http://%s/%s" % (self.hostname, api_call)

    def _call_api_no_response(self, api_call):
        """Call the given API command expecting no content in response.

        :param api_call: The API call without hostname and slash, e.g. "axis-cgi/param.cgi?camera=1".
        :type api_call: basestring
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network error or invalid HTTP status of the
                 API response
        """
        with closing(
                self._open_url(self._form_api_url(api_call), valid_statuses=[204], timeout=self.connection_timeout)):
            pass

    def _call_api_oneline_response(self, api_call):
        """Call the given API command expecting a one-line result.

        :param api_call: The API call without hostname and slash, e.g. "axis-cgi/param.cgi?camera=1".
        :type api_call: basestring
        :return: The response text line (without newline at the end).
        :rtype: :py:obj:`basestring`
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network error or invalid HTTP status of the
                 API response
        """
        return self._read_oneline_response(self._form_api_url(api_call), self.connection_timeout)

    def _call_api_multiline_response(self, api_call):
        """Call the given API command expecting a multiline result.

        :param api_call: The API call without hostname and slash, e.g. "axis-cgi/param.cgi?camera=1".
        :type api_call: basestring
        :return: The response text lines (without newlines at the end).
        :rtype: list of :py:obj:`basestring`
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network error or invalid HTTP status of the API response
        """
        return self._read_multiline_response(self._form_api_url(api_call), self.connection_timeout)

    def _call_api_binary_response(self, api_call):
        """Call the given API command expecting a binary result (e.g. an image).

        :param api_call: The API call without hostname and slash, e.g. "axis-cgi/param.cgi?camera=1".
        :type api_call: basestring
        :return: The response data.
        :rtype: :py:obj:`bytes`
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network error or invalid HTTP status of the API response
        """
        return self._read_binary_response(self._form_api_url(api_call), self.connection_timeout)

    def _call_ptz_command(self, command):
        """Call the given PTZ command.

        :param command: The command to execute. It is an ampersand-delimited string of type 'pan=1&tilt=5'.
        :type command: basestring
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network error or invalid HTTP status of the API response
        """

        rospy.logdebug("Calling PTZ command %s on host %s, camera %d" % (command, self.hostname, self.camera_id))
        self._call_api_no_response("axis-cgi/com/ptz.cgi?camera=%d&%s" % (self.camera_id, command))

    #################################################################
    # Static API that can be used before obtaining a VAPIX instance #
    #################################################################

    @staticmethod
    def _open_url(url, valid_statuses=None, timeout=2):
        """Open connection to Axis camera using HTTP

        :param url: The full URL to query.
        :type url: basestring
        :param valid_statuses: Status codes denoting valid responses. Other codes will raise an IOException. If None,
                               the reposnse status code is not examined.
        :type valid_statuses: list of int
        :param timeout: Timeout for the request.
        :type timeout: int
        :return: The stream with the response. The stream is never None (IOException would be thrown in such case).
        :rtype: :py:class:`urllib.addinfourl`
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network error or invalid HTTP status of the
                 API response
        """
        rospy.logdebug('Opening VAPIX URL %s .' % url)
        try:
            stream = urllib2.urlopen(url, timeout=timeout)
        except socket.timeout, e:  # sometimes the socket.timeout error is not correctly wrapped in URLError
            raise urllib2.URLError(e)

        if stream is not None:
            # check the response status
            if valid_statuses is None or stream.getcode() in valid_statuses:
                return stream
            else:
                if int(stream.getcode()) == 401:
                    raise IOError('Authentication required to access VAPIX URL %s . Either provide login credentials or'
                                  ' set up anonymous usage in the camera setup.' % url)
                elif int(stream.getcode()) == 200:
                    # if we e.g. expect only 204 No Content, and we receive something, it is an error
                    line = stream.readline().strip()
                    if line == "Error:":
                        line = stream.readline().strip()
                        raise RuntimeError('Request %s ended with error %s.' % (url, line))
                    else:
                        raise IOError(
                            'Received HTTP code %d in response to API request at URL %s .' % (stream.getcode(), url))
                else:
                    raise IOError(
                        'Received HTTP code %d in response to API request at URL %s .' % (stream.getcode(), url))
        else:
            raise IOError('Error opening URL %s .' % url)

    @staticmethod
    def _read_oneline_response(url, timeout=2):
        """Call the given full URL expecting a one-line response and return it.

        :param url: The full URL to query.
        :type url: basestring
        :param timeout: Timeout for the request.
        :type timeout: int
        :return: The response text line (without newline at the end).
        :rtype: :py:obj:`basestring`
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network error or invalid HTTP status of the
                 API response
        """
        with closing(VAPIX._open_url(url, valid_statuses=[200], timeout=timeout)) as response_stream:
            line = response_stream.readline()

            if line is None:
                raise IOError("Error reading response for API request at URL %s ." % url)

            if line.strip() == "Error:":
                line = response_stream.readline().strip()
                raise RuntimeError("API request %s returned error: %s" % (url, line))

            return line.strip("\n")

    @staticmethod
    def _read_multiline_response(url, timeout=2):
        """Call the given full URL expecting a multiline result.

        :param url: The full URL to query.
        :type url: basestring
        :param timeout: Timeout for the request.
        :type timeout: int
        :return: The response text lines (without newlines at the end).
        :rtype: list of :py:obj:`basestring`
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network error or invalid HTTP status of the
                 API response
        """
        with closing(VAPIX._open_url(url, valid_statuses=[200], timeout=timeout)) as response_stream:
            lines = []

            while not rospy.is_shutdown():
                line = response_stream.readline()

                if line is None or len(line) == 0 or line == "\n":
                    break

                if line.strip() == "Error:":
                    line = response_stream.readline().strip()
                    raise RuntimeError("API request %s returned error: %s" % (url, line))

                lines.append(line.strip())

            return lines

    @staticmethod
    def _read_binary_response(url, timeout=2):
        """Read a binary result for a given full URL (e.g. an image).

        :param url: The full URL to query.
        :type url: basestring
        :param timeout: Timeout for the API request.
        :type timeout: int
        :return: The response data.
        :rtype: :py:obj:`bytes`
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError`
        """
        with closing(VAPIX._open_url(url, valid_statuses=[200], timeout=timeout)) as response_stream:
            return response_stream.read()

    @staticmethod
    def parse_parameter_and_value_from_response_line(line):
        """Parse an API response line of the form key=value.

        :param line: The response line to parse.
        :type line: :py:obj:`str` | :py:obj:`unicode`
        :return: The parsed tuple (key, value).
        :rtype: tuple
        :raises: :py:exc:`ValueError` When the line cannot be parsed.
        """
        parts = line.split("=", 2)
        if len(parts) == 2:
            return parts[0].strip(), parts[1].strip()

        raise ValueError("Line '%s' is not a valid key-value parameter API reponse line." % line)

    @staticmethod
    def parse_list_parameter_value(list_value):
        """Parse an API response value that is a list and return the list.

        :param list_value: The value to be parsed as a (comma separated) list.
        :type list_value: :py:obj:`str` | :py:obj:`unicode`
        :return: The list of parsed values.
        :rtype: list of :py:obj:`basestring`
        """
        return list_value.split(",")

    @staticmethod
    def wakeup_camera(hostname, camera_id):
        """Try to wake up a camera in standby mode.

        :param hostname: Hostname of the camera.
        :type hostname: basestring
        :param camera_id: Id of the camera. Usually 1-4.
        :type camera_id: int
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError` on network error or invalid HTTP status of the
                 API response
        """
        # TODO what to do if there is no PTZ support?
        url = 'http://%s/axis-cgi/com/ptz.cgi?camera=%d&autofocus=on' % (hostname, camera_id)
        with closing(VAPIX._open_url(url, valid_statuses=[200, 204])):
            pass

    @staticmethod
    def setup_authentication(hostname, username, password, use_encrypted_password=False):
        """Set user credentials to the HTTP handler, so that if authentication is required, they will be used.

        :param hostname: Hostname of the camera.
        :type hostname: basestring
        :param username: Username.
        :type username: basestring
        :param password: Password.
        :type password: basestring
        :param use_encrypted_password: Whether to use Plain HTTP Auth (False) or Digest HTTP Auth (True).
        :type use_encrypted_password: bool
        """
        # create a password manager
        password_manager = urllib2.HTTPPasswordMgrWithDefaultRealm()

        # Add the username and password, use default realm.
        top_level_url = "http://%s" % hostname
        password_manager.add_password(None, top_level_url, username, password)

        if use_encrypted_password:
            handler = urllib2.HTTPDigestAuthHandler(password_manager)
        else:
            handler = urllib2.HTTPBasicAuthHandler(password_manager)

        # create "opener" (OpenerDirector instance)
        opener = urllib2.build_opener(handler)

        # ...and install it globally so it can be used with urlopen.
        urllib2.install_opener(opener)

    @staticmethod
    def get_api_for_camera(hostname, username=None, password=None, camera_id=1, use_encrypted_password=False):
        """Return the VAPIX API instance corresponding to the autodetected API version (both v2 and v3 are supported).

        :param hostname: Hostname of the camera (without http://, may be an IP address).
        :type hostname: basestring
        :param username: If login is needed, provide a username here.
        :type username: :py:obj:`basestring` | None
        :param password: If login is needed, provide a password here.
        :type password: :py:obj:`basestring` | None
        :param camera_id: ID (number) of the camera. Can be 1 to 4.
        :type camera_id: int
        :param use_encrypted_password: Whether to use Plain HTTP Auth (False) or Digest HTTP Auth (True).
        :type use_encrypted_password: bool
        :return: Autodetected API instance.
        :rtype: :py:class:`VAPIX`
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.ULRError`, :py:exc:`ValueError` If connecting to the API failed.
        :raises: :py:exc:`RuntimeError` If unexpected values are returned by the API.
        """
        # Enable HTTP login if a username and password are provided.
        if username is not None and len(username) > 0 and password is not None:
            rospy.logdebug("Using authentication credentials with user %s on host %s" % (username, hostname))
            VAPIX.setup_authentication(hostname, username, password, use_encrypted_password)

        rospy.logdebug("Starting VAPIX autodetection.")
        try:
            try:
                # First, try the v3 API. This URL is only valid for v3, so we strictly check for the version number.
                return VAPIX._get_v3_api(hostname, camera_id)
            except (IOError, ValueError):
                # Next, try the v2 API. This URL should only work in the version 2 API.
                try:
                    return VAPIX._get_v2_api(hostname, camera_id)
                except (IOError, ValueError):
                    # Maybe the camera is in standby mode. Try to wake it up.
                    rospy.logdebug("First try on VAPIX autodetection failed. The camera may be in standby mode. "
                                   "Trying to wake it up.")
                    VAPIX.wakeup_camera(hostname, camera_id)

                    # After waking up, try again both API v3 and v2
                    rospy.logdebug("Starting VAPIX autodetection - second try.")
                    try:
                        return VAPIX._get_v3_api(hostname, camera_id)
                    except (IOError, ValueError):
                        return VAPIX._get_v2_api(hostname, camera_id)
        except Exception as e:
            rospy.logerr("Could not autodetect or connect to VAPIX on host %s, camera %d. "
                         "The camera stream will be unavailable. Cause: %s" % (hostname, camera_id, str(e.args)))
            raise e

    @staticmethod
    def _get_v3_api(hostname, camera_id=1):
        """Check if the camera supports VAPIX v3, and if it does, return a corresponding VAPIXv3 instance.

        :param hostname: Hostname of the camera (without http://, may be an IP address)
        :type hostname: basestring
        :param camera_id: ID (number) of the camera. Can be 1 to 4.
        :type camera_id: int
        :return: The VAPIXv3 instance.
        :rtype: :py:class:`VAPIXv3`
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.ULRError`, :py:exc:`ValueError` If connecting to the API failed.
        :raises: :py:exc:`RuntimeError` If unexpected values are returned by the API.
        """
        response = VAPIX._read_oneline_response(
            "http://%s/axis-cgi/param.cgi?camera=%d&action=list&group=root.Properties.API.HTTP.Version" %
            (hostname, camera_id)
        )
        version = VAPIX.parse_parameter_and_value_from_response_line(response)[1]

        if version != "3":
            raise RuntimeError("Unexpected VAPIX version: %s" % version)

        rospy.loginfo("Autodetected VAPIX version 3 for communication with camera %d" % camera_id)
        return VAPIXv3(hostname, camera_id)

    @staticmethod
    def _get_v2_api(hostname, camera_id=1):
        """Check if the camera supports VAPIX v2, and if it does, return a corresponding VAPIXv2 instance.

        :param hostname: Hostname of the camera (without http://, may be an IP address)
        :type hostname: basestring
        :param camera_id: ID (number) of the camera. Can be 1 to 4.
        :type camera_id: int
        :return: The VAPIXv2 instance.
        :rtype: :py:class:`VAPIXv2`
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.ULRError`, :py:exc:`ValueError` If connecting to the API failed.
        :raises: :py:exc:`RuntimeError` If unexpected values are returned by the API.
        """
        response = VAPIX._read_oneline_response(
            "http://%s/axis-cgi/view/param.cgi?camera=%d&action=list&group=root.Properties.API.HTTP.Version" %
            (hostname, camera_id)
        )
        version = int(VAPIX.parse_parameter_and_value_from_response_line(response)[1])

        if not 0 < version < 3:
            raise RuntimeError("Unexpected VAPIX version: %s" % version)

        rospy.loginfo("Autodetected VAPIX version %d for communication with camera %d" % (version, camera_id))
        return VAPIXv2(hostname, camera_id)


class VAPIXv2(VAPIX):
    """A class representing the VAPIX API version 2.

    Most of the functionalities should be implemented in the parent class VAPIX.
    Here, only version-specific implementations have their place.
    """
    def __repr__(self):
        return "VAPIXv2(hostname=%r, camera_id=%r, connection_timeout=%r)" % (
            self.hostname, self.camera_id, self.connection_timeout)

    def _form_parameter_url(self, group):
        return self._form_api_url("axis-cgi/view/param.cgi?camera=%d&action=list&group=%s" % (self.camera_id, group))


class VAPIXv3(VAPIX):
    """A class representing the VAPIX API version 3.

    Most of the functionalities should be implemented in the parent class VAPIX.
    Here, only version-specific implementations have their place.
    """
    def __repr__(self):
        return "VAPIXv3(hostname=%r, camera_id=%r, connection_timeout=%r)" % (
            self.hostname, self.camera_id, self.connection_timeout)

    def _form_parameter_url(self, group):
        return self._form_api_url("axis-cgi/param.cgi?camera=%d&action=list&group=%s" % (self.camera_id, group))
