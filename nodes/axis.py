#!/usr/bin/env python

"""
Axis camera video driver. Inspired by:
https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/sandbox/axis_camera/axis.py

Communication with the camera is done using the Axis VAPIX API described at
http://www.axis.com/global/en/support/developer-support/vapix

.. note::

    This is a major rewrite of the former ros-drivers/axis_camera node, so it contains a (deprecated) backwards
    compatibility layer for the previous (non-released) API.
"""

import math
import re

import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo
import camera_info_manager
import dynamic_reconfigure.server

from axis_camera.cfg import VideoStreamConfig
from axis_camera.srv import TakeSnapshot, TakeSnapshotResponse
from axis_camera.vapix import VAPIX
from axis_camera.video_streaming import ImageStreamingThread
from axis_camera.dynamic_reconfigure_tools import change_enum_items

# BACKWARDS COMPATIBILITY LAYER
StreamThread = ImageStreamingThread  # deprecated


class Axis(rospy.SubscribeListener):
    """The ROS-VAPIX interface for video streaming."""

    def __init__(self, hostname, username, password, width, height, frame_id, camera_info_url, use_encrypted_password,
                 camera_id=1, auto_wakeup_camera=True, resolution_value=None, compression=0, fps=24, use_color=True,
                 use_square_pixels=False):
        """Create the ROS-VAPIX interface.

        :param hostname: Hostname of the camera (without http://, can be an IP address).
        :type hostname: basestring
        :param username: If login is needed, provide a username here.
        :type username: :py:obj:`basestring` | None
        :param password: If login is needed, provide a password here.
        :type password: :py:obj:`basestring` | None
        :param width: Width of the requested video stream in pixels (can be changed later). Must be one of the supported
                      resolutions.
        :type width: int
        :param height: Height of the requested video stream in pixels (can be changed later). Must be one of the
                       supported resolutions.
        :type height: int
        :param frame_id: The ROS TF frame assigned to the camera.
        :type frame_id: basestring
        :param camera_info_url: The URL pointing to the camera calaibration, if available.
        :type camera_info_url: basestring
        :param use_encrypted_password: Whether to use Plain HTTP Auth (False) or Digest HTTP Auth (True).
        :type use_encrypted_password: bool
        :param camera_id: ID (number) of the camera. Can be 1 to 4.
        :type camera_id: int
        :param auto_wakeup_camera: If True, there will be a wakeup trial after first unsuccessful network command.
        :type auto_wakeup_camera: bool
        :param resolution_value: The requested video resolution in the form `width`x`height`.
        :type resolution_value: basestring
        :param compression: Compression of the image (0 - no compression, 100 - max compression).
        :type compression: int
        :param fps: The desired frames per second.
        :type fps: int
        :param use_color: If True, send a color stream, otherwise send only grayscale image.
        :type use_color: bool
        :param use_square_pixels: If True, the resolution will be stretched to match 1:1 pixels.
                                  By default, the pixels have a ratio of 11:12.
        :type use_square_pixels: bool
        :raises: :py:exc:`ValueError` if the requested resolution (either the `resolution`, or `width`+`height`
                 is not supported.
        """
        # True every time the video parameters have changed and the URL has to be altered (set from other threads).
        self.video_params_changed = False

        self._hostname = hostname
        self._camera_id = camera_id

        self._api = None
        # autodetect the VAPIX API and connect to it; try it forever
        while self._api is None and not rospy.is_shutdown():
            try:
                self._api = VAPIX.get_api_for_camera(hostname, username, password, camera_id, use_encrypted_password)
            except (IOError, ValueError):
                rospy.loginfo("Retrying connection to VAPIX on host %s, camera %d in 2 seconds." %
                              (hostname, camera_id))
                rospy.sleep(2)
        if rospy.is_shutdown():
            return

        self._allowed_resolutions = self._get_allowed_resolutions()

        rospy.loginfo("The following resolutions are available for camera %d:\n%s" %
                      (camera_id, "\n".join([str(res) for res in self._allowed_resolutions])))
        rospy.set_param("~allowed_resolutions", [res.get_vapix_representation() for res in self._allowed_resolutions])

        # Sometimes the camera falls into power saving mode and stops streaming.
        # This setting allows the script to try to wake up the camera.
        self._auto_wakeup_camera = auto_wakeup_camera

        # dynamic-reconfigurable properties - definitions
        self._width = None  # deprecated
        self._height = None  # deprecated
        self._resolution = None
        self._compression = None
        self._fps = None
        self._use_color = None
        self._use_square_pixels = None

        # dynamic-reconfigurable properties - defaults
        if resolution_value is not None:
            self.set_resolution(resolution_value)
        else:
            resolution = self.find_resolution_by_size(width, height)
            self.set_resolution(resolution.get_vapix_representation())

        self.set_compression(compression)
        self.set_fps(fps)
        self.set_use_color(use_color)
        self.set_use_square_pixels(use_square_pixels)

        # only advertise the supported resolutions on dynamic reconfigure
        change_enum_items(
            VideoStreamConfig,
            "resolution",
            [{
                'name': res.name if isinstance(res, CIFVideoResolution) else str(res),
                'value': res.get_vapix_representation(),
                'description': str(res)
            } for res in self._allowed_resolutions],
            resolution_value
        )

        # dynamic reconfigure server
        self._video_stream_param_change_server = dynamic_reconfigure.server.Server(VideoStreamConfig,
                                                                                   self.reconfigure_video)

        # camera info setup
        self._frame_id = frame_id
        self._camera_info_url = camera_info_url

        # generate a valid camera name based on the hostname
        self._camera_name = camera_info_manager.genCameraName(self._hostname)
        self._camera_info = camera_info_manager.CameraInfoManager(cname=self._camera_name, url=self._camera_info_url)
        self._camera_info.loadCameraInfo()  # required before getCameraInfo()

        # the thread used for streaming images (is instantiated when the first image subscriber subscribes)
        self._streaming_thread = None

        # the publishers are started/stopped lazily in peer_subscribe/peer_unsubscribe
        self._video_publisher = rospy.Publisher("image_raw/compressed", CompressedImage, self, queue_size=100)
        self._camera_info_publisher = rospy.Publisher("camera_info", CameraInfo, self, queue_size=100)

        self._snapshot_server = rospy.Service("take_snapshot", TakeSnapshot, self.take_snapshot)

        # BACKWARDS COMPATIBILITY LAYER

        self.username = username  # deprecated
        self.password = password  # deprecated
        self.use_encrypted_password = use_encrypted_password  # deprecated
        self.st = None  # deprecated
        self.pub = self._video_publisher  # deprecated
        self.caminfo_pub = self._camera_info_publisher  # deprecated

    def __str__(self):
        (width, height) = self._resolution.get_resolution(self._use_square_pixels)
        return 'Axis driver on host %s, camera %d (%dx%d px @ %d FPS)' % \
               (self._hostname, self._api.camera_id, width, height, self._fps)

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        """Lazy-start the image-publisher."""
        if self._streaming_thread is None:
            self._streaming_thread = ImageStreamingThread(self)
            self._streaming_thread.start()
        else:
            self._streaming_thread.resume()

    def peer_unsubscribe(self, topic_name, num_peers):
        """Lazy-stop the image-publisher when nobody is interested"""
        if num_peers == 0:
            self._streaming_thread.pause()

    def take_snapshot(self, request):
        """Retrieve a snapshot from the camera.

        :param request: The service request.
        :type request: :py:class:`axis_camera.srv.TakeSnapshotRequest`
        :return: The response containing the image.
        :rtype: :py:class:`axis_camera.srv.TakeSnapshotResponse`
        :raises: :py:exc:`IOError`, :py:exc:`urllib2.URLError`
        """
        image_data = self._api.take_snapshot()

        image = CompressedImage()

        image.header.stamp = rospy.Time.now()
        image.header.frame_id = self._frame_id

        image.format = "jpeg"

        image.data = image_data

        response = TakeSnapshotResponse()
        response.image = image

        return response

    def reconfigure_video(self, config, level):
        """Dynamic reconfigure callback for video parameters.

        :param config: The requested configuration.
        :type config: dict
        :param level: Unused here.
        :type level: int
        :return: The config corresponding to what was really achieved.
        :rtype: dict
        """
        self.__try_set_value_from_config(config, 'compression', self.set_compression)
        self.__try_set_value_from_config(config, 'fps', self.set_fps)
        self.__try_set_value_from_config(config, 'use_color', self.set_use_color)
        self.__try_set_value_from_config(config, 'use_square_pixels', self.set_use_square_pixels)

        try:
            self.set_resolution(config.resolution)
        except ValueError:
            config.resolution = self._resolution.name

        return config

    def __try_set_value_from_config(self, config, field, setter):
        """First, try to call `setter(config[field])`, and if this call doesn't succeed. set the field in config to
        its value stored in this class.

        :param config: The dynamic reconfigure config dictionary.
        :type config: dict
        :param field: The field name (both in :py:obj:`config` and in :py:obj:`self`).
        :type field: basestring
        :param setter: The setter to use to set the value.
        :type setter: lambda function
        """
        try:
            setter(config[field])
        except ValueError:
            config[field] = getattr(self, field)

    #################################
    # DYNAMIC RECONFIGURE CALLBACKS #
    #################################

    def set_resolution(self, resolution_value):
        """Request a new resolution for the video stream.

        :param resolution_value: The string of type `width`x`height`.
        :type resolution_value: basestring
        :raises: :py:exc:`ValueError` if the resolution is unknown/unsupported.
        """
        if isinstance(resolution_value, basestring) and (
                self._resolution is None or resolution_value != self._resolution.get_vapix_representation()):
            self._resolution = self._get_resolution_from_param_value(resolution_value)
            self.video_params_changed = True
            # deprecated values
            self._width = self._resolution.get_resolution(self._use_square_pixels)[0]
            self._height = self._resolution.get_resolution(self._use_square_pixels)[1]

    def _get_resolution_from_param_value(self, value):
        """Return a :py:class:`VideoResolution` object corresponding to the given video resolution param string.

        :param value: Value of the resolution parameter to parse (of form `width`x`height`).
        :type value: basestring
        :return: The :py:class:`VideoResolution` corresponding to the given resolution param string.
        :rtype: :py:class:`VideoResolution`
        :raises: :py:exc:`ValueError` if the resolution is unknown/unsupported.
        """
        for resolution in self._allowed_resolutions:
            if resolution.get_vapix_representation() == value:
                return resolution

        raise ValueError("%s is not a valid valid resolution." % value)

    def find_resolution_by_size(self, width, height):
        """Return a :py:class:`VideoResolution` object with the given dimensions.

        If there are more resolutions with the same size, any of them may be returned.

        :param width: Image width in pixels.
        :type width: int
        :param height: Image height in pixels.
        :type height: int
        :return: The corresponding resolution object.
        :rtype: :py:class:`VideoResolution`
        :raises: :py:exc:`ValueError` if no resolution with the given dimensions can be found.
        """
        size_to_find = (width, height)
        for resolution in self._allowed_resolutions:
            size = resolution.get_resolution(use_square_pixels=False)
            if size == size_to_find:
                return resolution

            size = resolution.get_resolution(use_square_pixels=True)
            if size == size_to_find:
                return resolution

        raise ValueError("Cannot find a supported resolution with dimensions %dx%d" % size_to_find)

    def _get_allowed_resolutions(self):
        """Return a list of resolutions supported both by the camera.

        :return: The supported resolutions list.
        :rtype: list of :py:class:`VideoResolution`
        """
        camera_resolutions = self._get_resolutions_supported_by_camera()

        return camera_resolutions

    def _get_resolutions_supported_by_camera(self):
        """Return a list of resolutions supported the camera.

        :return: The supported resolutions list.
        :rtype: list of :py:class:`VideoResolution`
        """
        try:
            names = self._api.parse_list_parameter_value(self._api.get_parameter("Properties.Image.Resolution"))
            return [VideoResolution.parse_from_vapix_param_value(name, self._api) for name in names]
        except (IOError, ValueError):
            rospy.logwarn("Could not determine resolutions supported by the camera. Asssuming only CIF.")
            return [CIFVideoResolution("CIF", 384, 288)]

    def set_compression(self, compression):
        """Request the given compression level for the video stream.

        :param compression: Compression of the image (0 - no compression, 100 - max compression).
        :type compression: int
        :raises: :py:exc:`ValueError` if the given compression level is outside the allowed range.
        """
        if compression != self._compression:
            self._compression = self.sanitize_compression(compression)
            self.video_params_changed = True

    @staticmethod
    def sanitize_compression(compression):
        """Make sure the given value can be used as a compression level of the video stream.

        :param compression: Compression of the image (0 - no compression, 100 - max compression).
        :type compression: int
        :return: The given compression converted to an int.
        :rtype: int
        :raises: :py:exc:`ValueError` if the given compression level is outside the allowed range.
        """
        compression = int(compression)
        if not (0 <= compression <= 100):
            raise ValueError("%s is not a valid value for compression." % str(compression))

        return compression

    def set_fps(self, fps):
        """Request the given compression level for the video stream.

        :param fps: The desired frames per second.
        :type fps: int
        :raises: :py:exc:`ValueError` if the given FPS is outside the allowed range.
        """
        if fps != self._fps:
            self._fps = self.sanitize_fps(fps)
            self.video_params_changed = True

    @staticmethod
    def sanitize_fps(fps):
        """Make sure the given value can be used as FPS of the video stream.

        :param fps: The desired frames per second.
        :type fps: int
        :return: The given FPS converted to an int.
        :rtype: int
        :raises: :py:exc:`ValueError` if the given FPS is outside the allowed range.
        """
        fps = int(fps)
        if not (1 <= fps <= 30):
            raise ValueError("%s is not a valid value for FPS." % str(fps))

        return fps

    def set_use_color(self, use_color):
        """Request using/not using color in the video stream.
        :param use_color: If True, send a color stream, otherwise send only grayscale image.

        :type use_color: bool
        :raises: :py:exc:`ValueError` if the given argument is not a bool.
        """
        if use_color != self._use_color:
            self._use_color = self.sanitize_bool(use_color, "use_color")
            self.video_params_changed = True

    def set_use_square_pixels(self, use_square_pixels):
        """Request using/not using square pixels.

        :param use_square_pixels: If True, the resolution will be stretched to match 1:1 pixels.
                                  By default, the pixels have a ratio of 11:12.
        :type use_square_pixels: bool
        :raises: :py:exc:`ValueError` if the given argument is not a bool.
        """
        if use_square_pixels != self._use_square_pixels:
            self._use_square_pixels = self.sanitize_bool(use_square_pixels, "use_square_pixels")
            self.video_params_changed = True

    @staticmethod
    def sanitize_bool(value, field_name):
        """Convert the given value to a bool.

        :param value: Either True, False,, "1", "0", 1 or 0.
        :type value: :py:class:`basestring` | :py:class:`bool` | :py:class:`int`
        :param field_name: Name of the field this value belongs to (just for debug messages).
        :type field_name: basestring
        :return: The bool value of the given value.
        :rtype: :py:class:`bool`
        :raises: :py:exc:`ValueError` if the given value is not supported in this conversion.
        """
        if value not in (True, False, "1", "0", 1, 0):
            raise ValueError("%s is not a valid value for %s." % (str(value), field_name))

        # bool("0") returns True because it is a nonempty string
        if value == "0":
            return False

        return bool(value)


class VideoResolution(object):
    """A class representing a video resolution."""

    def __init__(self, width, height):
        """Create a representation of the resolution.

        :param width: Width of the resolution in pixels.
        :type width: int
        :param height: Height of the resolution in pixels.
        :type height: int
        """
        super(VideoResolution, self).__init__()

        self.width = int(width)
        self.height = int(height)

        self.square_pixel_conversion_ratio_width = 12.0 / 11.0
        self.square_pixel_conversion_ratio_height = 1

    def __str__(self):
        return "%dx%d" % (self.width, self.height)

    def __repr__(self):
        return "VideoResolution(width=%r,height=%r)" % (self.width, self.height)

    def get_resolution(self, use_square_pixels=False):
        """Get the image dimensions corresponding to this resolution.

        :param use_square_pixels: Whether to strech the resulting resolution to square pixels.
        :type use_square_pixels: bool
        :return: A tuple (width, height)
        :rtype: tuple
        """
        width = self.width
        height = self.height

        if use_square_pixels:
            width = int(math.ceil(self.square_pixel_conversion_ratio_width * self.width))
            height = int(math.ceil(self.square_pixel_conversion_ratio_height * self.height))

        return width, height

    def get_vapix_representation(self):
        return "%dx%d" % (self.width, self.height)

    @staticmethod
    def parse_from_vapix_param_value(value, api):
        assert isinstance(value, basestring)
        assert isinstance(api, VAPIX)

        numeric_regexp = re.compile(r"(\d+)x(\d+)")
        match = numeric_regexp.match(value)

        if match is not None:
            return VideoResolution(int(match.group(1)), int(match.group(2)))
        else:  # resolution given by CIF name
            name = value
            width, height = api.resolve_video_resolution_name(name)
            return CIFVideoResolution(name, width, height)


class CIFVideoResolution(VideoResolution):
    """A class representing a CIF standard resolution."""

    def __init__(self, name, width, height):
        """Create a representation of a CIF resolution.

        :param name: CIF standard name of the resolution.
        :type name: basestring
        :param width: Width of the resolution in pixels.
        :type width: int
        :param height: Height of the resolution in pixels.
        :type height: int
        """
        super(CIFVideoResolution, self).__init__(width, height)

        self.name = name

    def __str__(self):
        return "%s (%dx%d)" % (self.name, self.width, self.height)

    def __repr__(self):
        return "CIFVideoResolution(name=%r,width=%r,height=%r)" % (self.name, self.width, self.height)


def main():
    """Start the ROS driver and ROS node."""
    rospy.init_node("axis_driver")

    arg_defaults = {
        'hostname': '192.168.0.90',       # default IP address
        'username': None,               # default login name
        'password': None,
        'width': 704,
        'height': 576,
        'frame_id': 'axis_camera',
        'camera_info_url': '',
        'use_encrypted_password': False,
        'camera_id': 1,
        'auto_wakeup_camera': True,
        'resolution_value': '704x576',
        'compression': 0,
        'fps': 24,
        'use_color': True,
        'use_square_pixels': False,
        }
    args = read_args_with_defaults(arg_defaults)
    Axis(**args)
    rospy.spin()


def read_args_with_defaults(arg_defaults):
    """Look up parameters starting in the driver's private parameter space, but also searching outer namespaces.
    Defining them in a higher namespace allows the axis_ptz.py script to share parameters with the driver."""
    args = {}
    for name, val in arg_defaults.iteritems():
        full_name = rospy.search_param(name)
        if full_name is None:
            args[name] = val
        else:
            args[name] = rospy.get_param(full_name, val)
    # resolve frame_id with tf_prefix (unless already absolute)
    if args['frame_id'][0] != '/':        # not absolute?
        tf_prefix = rospy.search_param('tf_prefix')
        prefix_val = ''
        if tf_prefix is not None:           # prefix defined?
            prefix_val = rospy.get_param(tf_prefix)
            if prefix_val[0] != '/':          # prefix not absolute?
                prefix_val = '/' + prefix_val
        args['frame_id'] = prefix_val + '/' + args['frame_id']
    return args

if __name__ == "__main__":
    main()
