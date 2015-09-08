import urllib2
from contextlib import closing
from abc import ABCMeta, abstractmethod

import rospy

class VAPIX(object):
    __metaclass__ = ABCMeta

    def __init__(self, hostname, camera_id=1, connection_timeout=2):
        self.hostname = hostname
        self.camera_id = camera_id
        self.connection_timeout = connection_timeout

    @abstractmethod
    def get_parameter(self, name):
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

    def restart_camera(self):
        """
        Restart (re-initialize) the camera. This requires admin credentials to be given when creating this API instance.
        """
        rospy.loginfo("Restarting camera %d on %s ." % (self.camera_id, self.hostname))
        self._call_api_no_response("axis-cgi/admin/restart.cgi")
        rospy.sleep(10)

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
        with closing(self._open_url(self._form_api_url(api_call), valid_statuses=[200, 204, 304])):
            pass

    @staticmethod
    def _open_url(url, valid_statuses=None, timeout=2):
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
        :raises: IOException, urllib2.URLException
        """

        rospy.logdebug('Opening VAPIX URL %s .' % url)
        stream = urllib2.urlopen(url, timeout=timeout)

        if stream is not None:
            # check the response status
            if valid_statuses is None or stream.getcode() in valid_statuses:
                return stream
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
        :raises: IOException, urllib2.URLException
        """
        with closing(VAPIX._open_url(url, valid_statuses=[200, 204], timeout=timeout)) as response_stream:
            line = response_stream.readline()

            if line is None:
                raise IOError("Error reading response for API request at URL %s ." % url)

            return line.strip("\n")

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

        raise ValueError("Line %s is not a valid key-value parameter API reponse line." % line)

    @staticmethod
    def parse_list_parameter_value(list_value):
        return list_value.split(",")

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

    def get_parameter(self, name):
        url = self._form_api_url("axis-cgi/view/param.cgi?camera=%d&action=list&group=%s" % (self.camera_id, name))
        response_line = self._read_oneline_response(url, self.connection_timeout)
        value = self._parse_parameter_and_value_from_response_line(response_line)
        return value[1]


class VAPIXv3(VAPIX):
    def __repr__(self):
        return "VAPIX v3"

    def get_parameter(self, name):
        url = self._form_api_url("axis-cgi/param.cgi?camera=%d&action=list&group=%s" % (self.camera_id, name))
        response_line = self._read_oneline_response(url, self.connection_timeout)
        value = self._parse_parameter_and_value_from_response_line(response_line)
        return value[1]