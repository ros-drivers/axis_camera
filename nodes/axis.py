#!/usr/bin/env python

"""
Axis camera video driver. Inspired by:
https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/sandbox/axis_camera/axis.py

Communication with the camera is done using the Axis VAPIX API described at
http://www.axis.com/global/en/support/developer-support/vapix

This is a major rewrite of the former ros-drivers/axis_camera node, so it contains a backwards compatibility layer for
the previous API.
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

# BACKWARDS COMPATIBILITY LAYER
StreamThread = ImageStreamingThread  # deprecated


class Axis(rospy.SubscribeListener):
    """The ROS-VAPIX interface for video streaming."""

    def __init__(self, hostname, username, password, width, height, frame_id, camera_info_url, use_encrypted_password,
                 camera_id=1, auto_wakeup_camera=True, resolution_name=None, compression=0, fps=24, use_color=True,
                 use_square_pixels=False):
        """Create the ROS-VAPIX interface.
        :param hostname: Hostname of the camera (without http://, can be an IP address).
        :type hostname: basestring
        :param username: If login is needed, provide a username here.
        :type username: basestring|None
        :param password: If login is needed, provide a password here.
        :type password: basestring|None
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
        :param resolution_name: CIF name of the requested video resolution (e.g. '4CIF').
        :type resolution_name: basestring
        :param compression: Compression of the image (0 - no compression, 100 - max compression).
        :type compression: int
        :param fps: The desired frames per second.
        :type fps: int
        :param use_color: If True, send a color stream, otherwise send only grayscale image.
        :type use_color: bool
        :param use_square_pixels: If True, the resolution will be stretched to match 1:1 pixels.
                                  By default, the pixels have a ratio of 11:12.
        :type use_square_pixels: bool
        :raises: ValueError if the requested resolution (either the `resolution`, or `width`+`height` is not supported.
        """
        # True every time the video parameters have changed and the URL has to be altered (set from other threads).
        self.video_params_changed = False

        self.hostname = hostname
        self.camera_id = camera_id

        self.api = None
        # autodetect the VAPIX API and connect to it; try it forever
        while self.api is None and not rospy.is_shutdown():
            try:
                self.api = VAPIX.get_api_for_camera(hostname, username, password, camera_id, use_encrypted_password)
            except (IOError, ValueError):
                rospy.loginfo("Retrying connection to VAPIX on host %s, camera %d in 2 seconds." % (hostname, camera_id))
                rospy.sleep(2)
        if rospy.is_shutdown():
            return

        self.allowed_resolutions = self._get_allowed_resolutions()
        rospy.loginfo("The following resolutions are available for camera %d: %s" % (camera_id, repr(self.allowed_resolutions)))

        # Sometimes the camera falls into power saving mode and stops streaming.
        # This setting allows the script to try to wake up the camera.
        self.auto_wakeup_camera = auto_wakeup_camera

        # dynamic-reconfigurable properties - definitions
        self.width = None  # deprecated
        self.height = None  # deprecated
        self.resolution = None
        self.compression = None
        self.fps = None
        self.use_color = None
        self.use_square_pixels = None

        # dynamic-reconfigurable properties - defaults
        if resolution_name is not None:
            self.set_resolution(resolution_name)
        else:
            resolution = self.find_resolution_by_size(width, height)
            self.set_resolution(resolution.name)

        self.set_compression(compression)
        self.set_fps(fps)
        self.set_use_color(use_color)
        self.set_use_square_pixels(use_square_pixels)

        # dynamic reconfigure server
        self.video_stream_param_change_server = dynamic_reconfigure.server.Server(VideoStreamConfig, self.reconfigure_video)

        # camera info setup
        self.frame_id = frame_id
        self.camera_info_url = camera_info_url

        # generate a valid camera name based on the hostname
        self.camera_name = camera_info_manager.genCameraName(self.hostname)
        self.camera_info = camera_info_manager.CameraInfoManager(cname=self.camera_name, url=self.camera_info_url)
        self.camera_info.loadCameraInfo()  # required before getCameraInfo()

        # the thread used for streaming images (is instantiated when the first image subscriber subscribes)
        self.streaming_thread = None

        # the publishers are started/stopped lazily in peer_subscribe/peer_unsubscribe
        self.video_publisher = rospy.Publisher("image_raw/compressed", CompressedImage, self, queue_size=100)
        self.camera_info_publisher = rospy.Publisher("camera_info", CameraInfo, self, queue_size=100)

        self.snapshot_server = rospy.Service("take_snapshot", TakeSnapshot, self.take_snapshot)

        # BACKWARDS COMPATIBILITY LAYER

        self.username = username  # deprecated
        self.password = password  # deprecated
        self.use_encrypted_password = use_encrypted_password  # deprecated
        self.st = None  # deprecated
        self.pub = self.video_publisher  # deprecated
        self.caminfo_pub = self.camera_info_publisher  # deprecated

    def __str__(self):
        """Return string representation."""
        (width, height) = self.resolution.get_resolution(self.use_square_pixels)
        return 'Axis driver on host %s, camera %d (%dx%d px @ %d FPS)' % \
               (self.hostname, self.api.camera_id, width, height, self.fps)

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        """Lazy-start the image-publisher."""
        if self.streaming_thread is None:
            self.streaming_thread = ImageStreamingThread(self)
            self.streaming_thread.start()
        else:
            self.streaming_thread.resume()

    def peer_unsubscribe(self, topic_name, num_peers):
        """Lazy-stop the image-publisher when nobody is interested"""
        if num_peers == 0:
            self.streaming_thread.pause()

    def take_snapshot(self, request):
        """Retrieve a snapshot from the camera.
        :param request: The service request.
        :type request: TakeSnapshotRequest
        :return: The response containing the image.
        :rtype: TakeSnapshotResponse
        :raises: IOError, urllib2.URLError
        """
        image_data = self.api.take_snapshot()

        image = CompressedImage()

        image.header.stamp = rospy.Time.now()
        image.header.frame_id = self.frame_id

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
            config.resolution = self.resolution.name

        return config

    def __try_set_value_from_config(self, config, field, setter):
        """First, try to call `setter(config[field])`, and if this call doesn't succeed. set the field in config to
        its value stored in this class.
        :param config: The dynamic reconfigure config dictionary.
        :type config: dict
        :param field: The field name (both in `config` and in `self`).
        :type field: basestring
        :param setter: The setter to use to set the value.
        :type setter: lambda value
        """
        try:
            setter(config[field])
        except ValueError:
            config[field] = getattr(self, field)

    def set_resolution(self, resolution_name):
        """Request a new resolution for the video stream.
        :param resolution_name: The CIF standard name of the resolution. E.g. '4CIF'.
        :type resolution_name: basestring
        :raises: ValueError if the resolution is unknown/unsupported.
        """
        if isinstance(resolution_name, basestring) and (
                self.resolution is None or resolution_name != self.resolution.name):
            self.resolution = self._get_resolution_for_name(resolution_name)
            self.video_params_changed = True
            # deprecated values
            self.width = self.resolution.get_resolution(self.use_square_pixels)[0]
            self.height = self.resolution.get_resolution(self.use_square_pixels)[1]

    def _get_resolution_for_name(self, resolution_name):
        """Return a CIFVideoResolution object corresponding to the given CIF resolution name.
        :param resolution_name: The CIF standard name of the resolution. E.g. '4CIF'.
        :type resolution_name: basestring
        :return: The CIFVideoResolution corresponding to the given resolution name.
        :rtype: CIFVideoResolution
        :raises: ValueError if the resolution is unknown/unsupported.
        """
        if resolution_name not in self.allowed_resolutions:
            raise ValueError("%s is not a valid valid resolution." % resolution_name)

        return self.allowed_resolutions[resolution_name]

    def find_resolution_by_size(self, width, height):
        """Return a CIFVideoResolution object with the given dimensions.

        If there are more resolutions with the same size, any of them may be returned.
        :param width: Image width in pixels.
        :type width: int
        :param height: Image height in pixels.
        :type height: int
        :return: The corresponding resolution object.
        :rtype: CIFVideoResolution
        :raises: ValueError if no resolution with the given dimensions can be found.
        """
        size_to_find = (width, height)
        for resolution in self.allowed_resolutions.values():
            size = resolution.get_resolution(use_square_pixels=False)
            if size == size_to_find:
                return resolution

            size = resolution.get_resolution(use_square_pixels=True)
            if size == size_to_find:
                return resolution

        raise ValueError("Cannot find a supported resolution with dimensions %dx%d" % size_to_find)

    def _get_allowed_resolutions(self):
        """Return a dict (resolution_name=>resolution) of resolutions supported both by this implementation and the
        camera.
        :return: The supported resolutions dictionary.
        :rtype: dict(basestring => basestring)
        """
        config_resolutions = self._read_resolutions_from_config()
        camera_resolutions = self._get_resolutions_supported_by_camera()

        return dict((k, v) for k, v in config_resolutions.iteritems() if k in camera_resolutions)

    @staticmethod
    def _read_resolutions_from_config():
        """Return a dict (resolution_name=>resolution) of resolutions supported by the VideoStream.cfg config.
        :return: The supported resolutions dictionary.
        :rtype: dict(basestring => basestring)
        """
        video_stream_parameters = VideoStreamConfig.config_description['parameters']
        resolutions = dict()

        resolution_size_parser = re.compile(r'\((?P<width>[0-9]+)x(?P<height>[0-9]+)\)')

        # TODO: is there a better way than parsing and eval'ing the description?
        for parameter in video_stream_parameters:
            if parameter['name'] == "resolution":
                resolution_descriptions = eval(parameter['edit_method'])['enum']
                for resolution_description in resolution_descriptions:
                    resolution_name = resolution_description['name']

                    match_result = resolution_size_parser.search(resolution_description['description'])
                    if match_result is not None:
                        resolution_size = (match_result.group('width'), match_result.group('height'))
                    else:
                        rospy.logwarn("Invalid resolution found in VideoStream.cfg: %s" % resolution_description['description'])
                        continue

                    resolution = CIFVideoResolution(resolution_name, resolution_size[0], resolution_size[1])
                    resolutions[resolution_name] = resolution

                break

        if len(resolutions) > 0:
            return resolutions
        else:
            # return a default set of resolutions if we couldn't parse the cfg file for some reason
            return {
                '4CIF': CIFVideoResolution('4CIF', 704, 576),
                'CIF': CIFVideoResolution('CIF', 352, 288),
                'QCIF': CIFVideoResolution('QCIF', 176, 144),
            }

    def _get_resolutions_supported_by_camera(self):
        """Return a list of names of resolutions supported the camera.
        :return: The supported resolutions list.
        :rtype: list
        """
        try:
            return self.api.parse_list_parameter_value(self.api.get_parameter("root.Properties.Image.Resolution"))
        except (IOError, ValueError):
            rospy.logwarn("Could not determine resolutions supported by the camera. Asssuming only CIF.")
            return ["CIF"]

    def set_compression(self, compression):
        """Request the given compression level for the video stream.
        :param compression: Compression of the image (0 - no compression, 100 - max compression).
        :type compression: int
        :raises: ValueError if the given compression level is outside the allowed range.
        """
        if compression != self.compression:
            self.compression = self.sanitize_compression(compression)
            self.video_params_changed = True

    @staticmethod
    def sanitize_compression(compression):
        """Make sure the given value can be used as a compression level of the video stream.
        :param compression: Compression of the image (0 - no compression, 100 - max compression).
        :type compression: int
        :return: The given compression converted to an int.
        :raises: ValueError if the given compression level is outside the allowed range.
        """
        compression = int(compression)
        if not (0 <= compression <= 100):
            raise ValueError("%s is not a valid value for compression." % str(compression))

        return compression

    def set_fps(self, fps):
        """Request the given compression level for the video stream.
        :param fps: The desired frames per second.
        :type fps: int
        :raises: ValueError if the given FPS is outside the allowed range.
        """
        if fps != self.fps:
            self.fps = self.sanitize_fps(fps)
            self.video_params_changed = True

    @staticmethod
    def sanitize_fps(fps):
        """Make sure the given value can be used as FPS of the video stream.
        :param fps: The desired frames per second.
        :type fps: int
        :return: The given FPS converted to an int.
        :raises: ValueError if the given FPS is outside the allowed range.
        """
        fps = int(fps)
        if not (1 <= fps <= 30):
            raise ValueError("%s is not a valid value for FPS." % str(fps))

        return fps

    def set_use_color(self, use_color):
        """Request using/not using color in the video stream.
        :param use_color: If True, send a color stream, otherwise send only grayscale image.
        :type use_color: bool
        :raises: ValueError if the given argument is not a bool.
        """
        if use_color != self.use_color:
            self.use_color = self.sanitize_bool(use_color, "use_color")
            self.video_params_changed = True

    def set_use_square_pixels(self, use_square_pixels):
        """Request using/not using square pixels.
        :param use_square_pixels: If True, the resolution will be stretched to match 1:1 pixels.
                                  By default, the pixels have a ratio of 11:12.
        :type use_square_pixels: bool
        :raises: ValueError if the given argument is not a bool.
        """
        if use_square_pixels != self.use_square_pixels:
            self.use_square_pixels = self.sanitize_bool(use_square_pixels, "use_square_pixels")
            self.video_params_changed = True

    @staticmethod
    def sanitize_bool(value, field_name):
        """Convert the given value to a bool.
        :param value: Either True, False,, "1", "0", 1 or 0.
        :type value: basestring|bool|int
        :param field_name: Name of the field this value belongs to (just for debug messages).
        :type field_name: basestring
        :return: The bool value of the given value.
        :rtype: bool
        :raises: ValueError if the given value is not supported in this conversion.
        """
        if value not in (True, False, "1", "0", 1, 0):
            raise ValueError("%s is not a valid value for %s." % (str(value), field_name))

        # bool("0") returns True because it is a nonempty string
        if value == "0":
            return False

        return bool(value)


class CIFVideoResolution(object):
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
        super(CIFVideoResolution, self).__init__()

        self.name = name
        self.width = int(width)
        self.height = int(height)

        self.square_pixel_conversion_ratio_width = 12.0 / 11.0
        self.square_pixel_conversion_ratio_height = 1

    def __str__(self):
        return "%s (%dx%d)" % (self.name, self.width, self.height)

    def __repr__(self):
        return "CIFVideoResolution(name=%r,width=%r,height=%r)" % (self.name, self.width, self.height)

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
        'resolution_name': 'CIF',
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
