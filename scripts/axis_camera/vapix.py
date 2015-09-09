import urllib2
from contextlib import closing
from abc import ABCMeta, abstractmethod

import rospy


class Range(object):
    def __init__(self, min=float('-inf'), max=float('inf'), period=None):
        self.min = min
        self.max = max
        self.period = period

    def _normalize(self, value):
        if self.period is None:
            return value

        result = value
        while result > self.period/2.0:
            result -= self.period
        while result < -self.period / 2.0:
            result += self.period
        return result

    def is_in_range(self, value):
        return self.min <= self._normalize(value) <= self.max

    def crop_value(self, value):
        value = self._normalize(value)
        if value > self.max:
            return self.max
        elif value < self.min:
            return self.min
        else:
            return value

    def merge(self, range):
        assert isinstance(range, Range)

        period = None
        if self.period is not None:
            if range.period is not None:
                period = max(self.period, range.period)  # TODO: better handling in case of different periods
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
        return self.__str__()


class PTZLimit(object):
    def __init__(self, absolute=None, relative=None, velocity=None):
        self.absolute = absolute if absolute is not None else Range()
        self.relative = relative if relative is not None else Range()
        self.velocity = velocity if velocity is not None else Range()

    def merge(self, limit):
        assert isinstance(limit, PTZLimit)

        return PTZLimit(
            self.absolute.merge(limit.absolute),
            self.relative.merge(limit.relative),
            self.velocity.merge(limit.velocity)
        )

    def __str__(self):
        return "[absolute=%s, relative=%s, velocity=%s]" % (str(self.absolute), str(self.relative), str(self.velocity))

    def __repr__(self):
        return self.__str__()


class VAPIX(object):
    __metaclass__ = ABCMeta

    def __init__(self, hostname, camera_id=1, connection_timeout=5):
        self.hostname = hostname
        self.camera_id = camera_id
        self.connection_timeout = connection_timeout

        self._has_ptz = self.has_ptz()
        if self._has_ptz:
            self._ptz_capabilities = self.get_ptz_capabilities()
            self._ptz_limits = self.get_ptz_limits()

    @abstractmethod
    def _form_parameter_url(self, group):
        pass

    def get_video_stream(self, fps=24, resolution_name='CIF', compression=0, use_color=True, use_square_pixels=False):
        api_call = 'axis-cgi/mjpg/video.cgi?camera=%d&fps=%d&resolution=%s&compression=%d&color=%d&squarepixel=%d' % (
            self.camera_id, fps, resolution_name, compression, 1 if use_color else 0, 1 if use_square_pixels else 0
        )

        url = self._form_api_url(api_call)
        return self._open_url(url, valid_statuses=[200])

    def is_video_ok(self):
        url = self._form_api_url("axis-cgi/view/videostatus.cgi?status=%d" % self.camera_id)
        response_line = self._read_oneline_response(url, self.connection_timeout)
        status = self._parse_parameter_and_value_from_response_line(response_line)[1]

        return status == "video"

    def get_camera_position(self, get_zoom=True, get_focus=True, get_iris=True):
        """
        Get current camera position using the Axis VAPIX PTZ API
        :param get_zoom: If true, also try to get zoom. Set to false if using with no-zoom cameras.
        :type get_zoom: bool
        :param get_focus: If true, also try to get autofocus and focus. Set to false if using with no-focus cameras. Value True is deprecated.
        :type get_focus: bool
        :param get_iris: If true, also try to get autoiris and iris. Set to false if using with no-iris cameras. Value True is deprecated.
        :type get_iris: bool
        :return: The current camera position.
        :rtype: dict {'pan': ..., 'tilt': ..., ['zoom': ...]}
        :raises: IOError, urllib2.URLError
        """
        url = self._form_api_url("axis-cgi/com/ptz.cgi?query=position&camera=%d" % self.camera_id)
        response_lines = self._read_multiline_response(url, self.connection_timeout)

        position_keys = set(['pan', 'tilt'])
        if get_zoom:
            position_keys.add('zoom')
        if get_focus:  # deprecated
            position_keys.add('autofocus')
            position_keys.add('focus')
        if get_iris:  # deprecated
            position_keys.add('autoiris')
            position_keys.add('iris')

        position = dict()
        for line in response_lines:
            (key, value) = self._parse_parameter_and_value_from_response_line(line)
            if key in position_keys:
                if not key.startswith('auto'):
                    position[key] = float(value)
                else:
                    position[key] = True if value == 'on' else False

        if 'pan' not in position or 'tilt' not in position or (get_zoom and 'zoom' not in position):
            rospy.logdebug('Only succeeded to parse the following position values: %s' % repr(position))
            raise RuntimeError('Error requesting the current position of the camera. Unexpected API response.')

        return position

    def restart_camera(self):
        """
        Restart (re-initialize) the camera. This requires admin credentials to be given when creating this API instance.
        """
        rospy.loginfo("Restarting camera %d on %s ." % (self.camera_id, self.hostname))
        self._call_api_no_response("axis-cgi/admin/restart.cgi")

        # and now try to reload camera capabilities which might have changed after restart
        # (here we also just wait until the camera is ready again)
        while not rospy.is_shutdown():
            try:
                self._has_ptz = self.has_ptz()
                if self._has_ptz:
                    self._ptz_capabilities = self.get_ptz_capabilities()
                    self._ptz_limits = self.get_ptz_limits()
                break
            except (IOError, ValueError, RuntimeError):
                rospy.logdebug("Cannot get camera capabilities. Waiting 1s.")
                rospy.sleep(1)


    # PTZ movement
    def move_ptz_absolute(self, pan=None, tilt=None, zoom=None):
        self.require_capabilities('AbsolutePan', 'AbsoluteTilt', 'AbsoluteZoom')

        pan = (self._ptz_limits['Pan'].absolute.crop_value(pan) if pan is not None else None)
        tilt = (self._ptz_limits['Tilt'].absolute.crop_value(tilt) if tilt is not None else None)
        zoom = (self._ptz_limits['Zoom'].absolute.crop_value(zoom) if zoom is not None else None)

        commands = []
        if pan is not None:
            commands.append("pan=%f" % pan)
        if tilt is not None:
            commands.append("tilt=%f" % tilt)
        if zoom is not None:
            commands.append("zoom=%d" % zoom)

        if len(commands) > 0:
            command = "&".join(commands)
            self._call_ptz_command(command)

        return pan, tilt, zoom

    def move_ptz_relative(self, pan=None, tilt=None, zoom=None):
        self.require_capabilities('RelativePan', 'RelativeTilt', 'RelativeZoom')

        pan = (self._ptz_limits['Pan'].relative.crop_value(pan) if pan is not None else None)
        tilt = (self._ptz_limits['Tilt'].relative.crop_value(tilt) if tilt is not None else None)
        zoom = (self._ptz_limits['Zoom'].relative.crop_value(zoom) if zoom is not None else None)

        commands = []
        if pan is not None:
            commands.append("rpan=%f" % pan)
        if tilt is not None:
            commands.append("rtilt=%f" % tilt)
        if zoom is not None:
            commands.append("rzoom=%d" % zoom)

        if len(commands) > 0:
            command = "&".join(commands)
            self._call_ptz_command(command)

        return pan, tilt, zoom

    def set_ptz_velocity(self, pan=None, tilt=None, zoom=None):
        self.require_capabilities('ContinuousPan', 'ContinuousTilt', 'ContinuousZoom')

        pan = (self._ptz_limits['Pan'].velocity.crop_value(pan) if pan is not None else None)
        tilt = (self._ptz_limits['Tilt'].velocity.crop_value(tilt) if tilt is not None else None)
        zoom = (self._ptz_limits['Zoom'].velocity.crop_value(zoom) if zoom is not None else None)

        commands = []
        if pan is not None:
            if tilt is not None:
                commands.append("continuouspantiltmove=%d,%d" % (pan, tilt))
            else:
                commands.append("continuouspantiltmove=%d,0" % pan)
        elif tilt is not None:
            commands.append("continuouspantiltmove=0,%d" % tilt)
        if zoom is not None:
            commands.append("continuouszoommove=%d" % zoom)

        if len(commands) > 0:
            command = "&".join(commands)
            self._call_ptz_command(command)

        return pan, tilt, zoom

    # Focus
    def use_autofocus(self, use):
        self.require_capabilities('AutoFocus')
        self._call_ptz_command("autofocus=%s" % "on" if use else "off")
        return use

    def set_focus(self, focus):
        self.require_capabilities('AbsoluteFocus')
        focus = self._ptz_limits['Focus'].absolute.crop_value(focus)
        self._call_ptz_command("autofocus=off&focus=%d" % focus)
        return focus

    def adjust_focus(self, amount):
        self.require_capabilities('RelativeFocus')
        amount = self._ptz_limits['Focus'].relative.crop_value(amount)
        self._call_ptz_command("autofocus=off&rfocus=%d" % amount)
        return amount

    def set_focus_velocity(self, velocity):
        self.require_capabilities('ContinuousFocus')
        velocity = self._ptz_limits['Focus'].velocity.crop_value(velocity)
        self._call_ptz_command("autofocus=off&countinuousfocusmove=%d" % velocity)
        return velocity

    # Iris
    def use_autoiris(self, use):
        self.require_capabilities('AutoIris')
        self._call_ptz_command("autoiris=%s" % "on" if use else "off")
        return use

    def set_iris(self, iris):
        self.require_capabilities('AbsoluteIris')
        iris = self._ptz_limits['Iris'].absolute.crop_value(iris)
        self._call_ptz_command("autoiris=off&iris=%d" % iris)
        return iris

    def adjust_iris(self, amount):
        self.require_capabilities('RelativeIris')
        amount = self._ptz_limits['Iris'].relative.crop_value(amount)
        self._call_ptz_command("autoiris=off&riris=%d" % amount)
        return amount

    def set_iris_velocity(self, velocity):
        self.require_capabilities('ContinuousIris')
        velocity = self._ptz_limits['Iris'].velocity.crop_value(velocity)
        self._call_ptz_command("autoiris=off&countinuousirismove=%d" % velocity)
        return velocity

    # Brightness
    def set_brightness(self, brightness):
        self.require_capabilities('AbsoluteBrightness')
        brightness = self._ptz_limits['Brightness'].absolute.crop_value(brightness)
        self._call_ptz_command("brightness=%d" % brightness)
        return brightness

    def adjust_brightness(self, amount):
        self.require_capabilities('RelativeBrightness')
        amount = self._ptz_limits['Brightness'].relative.crop_value(amount)
        self._call_ptz_command("rbrightness=%d" % amount)
        return amount

    def set_brightness_velocity(self, velocity):
        self.require_capabilities('ContinuousBrightness')
        velocity = self._ptz_limits['Brightness'].velocity.crop_value(velocity)
        self._call_ptz_command("countinuousbrightnessmove=%d" % velocity)
        return velocity

    def _call_ptz_command(self, command):
        rospy.logdebug("Calling PTZ command %s on host %s, camera %d" % (command, self.hostname, self.camera_id))
        self._call_api_no_response("axis-cgi/com/ptz.cgi?camera=%d&%s" % (self.camera_id, command))

    # Backlight compensation
    def use_backlight_compensation(self, use):
        self.require_capabilities('BackLight')
        self._call_ptz_command("backlight=%s" % ("on" if use else "off"))
        return use

    # Infrared filter usage ("auto" value signalized by use=None)
    def use_ir_cut_filter(self, use):
        self.require_capabilities('IrCutFilter')
        self._call_ptz_command("ircutfilter=%s" % ("auto" if use is None else ("on" if use else "off")))
        return use

    # PTZ presence checks
    def has_ptz(self):
        return self.get_parameter("root.Properties.PTZ.PTZ") == "yes"

    def is_digital_ptz(self):
        return self.get_parameter("root.Properties.PTZ.DigitalPTZ") == "yes"

    # Camera and PTZ capabilities

    def has_capability(self, capability):
        if not self._has_ptz:
            return False
        return capability in self._ptz_capabilities

    def has_capabilities(self, *capabilities):
        if not self._has_ptz:
            return False

        for capability in capabilities:
            if not self.has_capability(capability):
                return False

        return True

    def require_capabilities(self, *capabilities):
        for capability in capabilities:
            if not self.has_capability(capability):
                raise RuntimeError("Camera %d on host %s doesn't support the required capability %s." % (
                    self.camera_id, self.hostname, capability))

    def get_ptz_capabilities(self):
        prefix = "root.PTZ.Support.S%d" % self.camera_id
        parameters = self.get_parameter_list(prefix)

        result = set()
        for (key, value) in parameters.iteritems():
            if value == "true":
                key_stripped = key.replace(prefix + ".", "")
                result.add(key_stripped)

        return result

    def get_ptz_limits(self):
        real_limits = self.get_real_ptz_limits()
        api_limits = self.get_api_ptz_limits()

        result = dict()
        keys = set().union(real_limits.keys()).union(api_limits.keys())
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

    # PTZ limits
    def get_real_ptz_limits(self):
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
    def get_api_ptz_limits():
        return {
            'Pan': PTZLimit(absolute=Range(-180, 180, 360), relative=Range(-360, 360, 720), velocity=Range(-100, 100)),
            'Tilt': PTZLimit(absolute=Range(-180, 180, 360), relative=Range(-360, 360, 720), velocity=Range(-100, 100)),
            'Zoom': PTZLimit(absolute=Range(1, 9999), relative=Range(-9999, 9999), velocity=Range(-100, 100)),
            'Focus': PTZLimit(absolute=Range(1, 9999), relative=Range(-9999, 9999), velocity=Range(-100, 100)),
            'Iris': PTZLimit(absolute=Range(1, 9999), relative=Range(-9999, 9999), velocity=Range(-100, 100)),
            'Brightness': PTZLimit(absolute=Range(1, 9999), relative=Range(-9999, 9999), velocity=Range(-100, 100)),
        }

    def get_parameter(self, name):
        url = self._form_parameter_url(name)
        response_line = self._read_oneline_response(url, self.connection_timeout)
        value = self._parse_parameter_and_value_from_response_line(response_line)
        return value[1]

    def get_parameter_list(self, group):
        url = self._form_parameter_url(group)
        response_lines = self._read_multiline_response(url, self.connection_timeout)

        result = dict()
        for response_line in response_lines:
            parameter_and_value = self._parse_parameter_and_value_from_response_line(response_line)
            result[parameter_and_value[0]] = parameter_and_value[1]

        return result

    # HELPER FUNCTIONS

    def _form_api_url(self, api_call):
        """
        Return the URL to be called to execute the given API call.
        :param api_call: The API call without hostname and slash, e.g. "axis-cgi/param.cgi?camera=1".
        :type api_call: basestring
        :return: The URL.
        :rtype: basestring
        """
        return "http://%s/%s" % (self.hostname, api_call)

    def _call_api_no_response(self, api_call):
        with closing(self._open_url(self._form_api_url(api_call), valid_statuses=[204], timeout=self.connection_timeout)):
            pass

    @staticmethod
    def _open_url(url, valid_statuses=None, timeout=5):
        """
        Open connection to Axis camera using HTTP

        :param url: The API URL to query.
        :type url: basestring
        :param valid_statuses: Status codes denoting valid responses. Other codes will raise an IOException.
        :type valid_statuses: list
        :param timeout: Timeout for the API request.
        :type timeout: int
        :return: The stream with the response. The stream is never None (IOException would be thrown in such case).
        :rtype urllib.addinfourl:
        :raises: IOError, urllib2.URLError
        """

        rospy.logdebug('Opening VAPIX URL %s .' % url)
        stream = urllib2.urlopen(url, timeout=timeout)

        if stream is not None:
            # check the response status
            if valid_statuses is None or stream.getcode() in valid_statuses:
                return stream
            else:
                if int(stream.getcode) == 401:
                    raise IOError('Authentication required to access VAPIX URL %s . Either provide login credentials or'
                                  ' set up anonymous usage in the camera setup.' % url)
                else:
                    raise IOError('Received HTTP code %d in response to API request at URL %s .' % (stream.getcode, url))
        else:
            raise IOError('Error opening URL %s .' % url)

    @staticmethod
    def _read_oneline_response(url, timeout=2):
        """
        Read a standard one-line result for a given API request.
        :param url: The API URL to query.
        :type url: basestring
        :param timeout: Timeout for the API request.
        :type timeout: int
        :return: The response text line (without newline at the end).
        :rtype: basestring
        :raises: IOError, urllib2.URLError
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
        """
        Read a standard multiline result for a given API request.
        :param url: The API URL to query.
        :type url: basestring
        :param timeout: Timeout for the API request.
        :type timeout: int
        :return: The response text lines (without newline at the end).
        :rtype: list
        :raises: IOError, urllib2.URLError
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
    def _parse_parameter_and_value_from_response_line(line):
        """
        Parse an API response line of the form key=value.
        :param line: The response line to parse.
        :type line: basestring
        :return: The parsed tuple (key, value).
        :rtype: tuple
        :raises: ValueError When the line cannot be parsed.
        """
        parts = line.split("=", 2)
        if len(parts) == 2:
            return parts[0].strip(), parts[1].strip()

        raise ValueError("Line '%s' is not a valid key-value parameter API reponse line." % line)

    @staticmethod
    def _parse_list_parameter_value(list_value):
        return list_value.split(",")

    # Static API that can be used before obtaining a VAPIX instance

    @staticmethod
    def wakeup_camera(hostname, camera_id):
        # TODO what to do if there is no PTZ support?
        url = 'http://%s/axis-cgi/com/ptz.cgi?camera=%d&autofocus=on' % (hostname, camera_id)
        with closing(VAPIX._open_url(url, valid_statuses=[200, 204])):
            pass

    @staticmethod
    def setup_authentication(hostname, username, password, use_encrypted_password=False):
        """only try to authenticate if user/pass configured.  I have not
        used this method (yet)."""

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
        """
        Return the VAPIX API instance corresponding to the autodetected API version (both v2 and v3 are supported).
        :param hostname: Hostname of the camera (without http://, may be an IP address).
        :type hostname: basestring
        :param username: If login is needed, provide a username here.
        :type username: basestring|None
        :param password: If login is needed, provide a password here.
        :type password: basestring|None
        :param camera_id: Id (number) of the camera. Can be 1 to 4.
        :type camera_id: int
        :param use_encrypted_password: If True, HTTP auth will use the Digest method, otherwise Basic will be used.
        :type use_encrypted_password: bool
        :return: Autodetected API instance.
        :rtype: VAPIX
        :raises: IOError, urllib2.ULRError, ValueError If connecting to the API failed.
        :raises: RuntimeError If unexpected values are returned by the API.
        """

        # Enable HTTP login if a username and password are provided.
        if username is not None and password is not None:
            rospy.logdebug("Using authentication credentials with user %s on host %s" % (username, hostname))
            VAPIX.setup_authentication(hostname, username, password, use_encrypted_password)

        rospy.logdebug("Starting VAPIX autodetection.")
        try:
            try:
                # First, try the v3 API. This URL is only valid for version 3, so we strictly check for the version number.
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
        response = VAPIX._read_oneline_response(
            "http://%s/axis-cgi/param.cgi?camera=%d&action=list&group=root.Properties.API.HTTP.Version" %
            (hostname, camera_id)
        )
        version = VAPIX._parse_parameter_and_value_from_response_line(response)[1]

        if version != "3":
            raise RuntimeError("Unexpected VAPIX version: %s" % version)

        rospy.loginfo("Autodetected VAPIX version 3 for communication with camera %d" % camera_id)
        return VAPIXv3(hostname, camera_id)

    @staticmethod
    def _get_v2_api(hostname, camera_id=1):
        response = VAPIX._read_oneline_response(
            "http://%s/axis-cgi/view/param.cgi?camera=%d&action=list&group=root.Properties.API.HTTP.Version" %
            (hostname, camera_id)
        )
        version = int(VAPIX._parse_parameter_and_value_from_response_line(response)[1])

        if not 0 < version < 3:
            raise RuntimeError("Unexpected VAPIX version: %s" % version)

        rospy.loginfo("Autodetected VAPIX version %d for communication with camera %d" % (version, camera_id))
        return VAPIXv2(hostname, camera_id)


class VAPIXv2(VAPIX):
    def __repr__(self):
        return "VAPIX v2"

    def _form_parameter_url(self, group):
        return self._form_api_url("axis-cgi/view/param.cgi?camera=%d&action=list&group=%s" % (self.camera_id, group))


class VAPIXv3(VAPIX):
    def __repr__(self):
        return "VAPIX v3"

    def _form_parameter_url(self, group):
        return self._form_api_url("axis-cgi/param.cgi?camera=%d&action=list&group=%s" % (self.camera_id, group))
