#!/usr/bin/env python
#
# Axis camera image driver. Based on:
# https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/sandbox/axis_camera
# /axis.py
#
# Communication with the camera is done using the Axis VAPIX API described at
# http://www.axis.com/global/en/support/developer-support/vapix
#

import math
import re

import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo
import camera_info_manager
import dynamic_reconfigure.server
from axis_camera.cfg import VideoStreamConfig

from axis_camera.vapix import VAPIX
from axis_camera.streaming import ImageStreamingThread

# BACKWARDS COMPATIBILITY LAYER
StreamThread = ImageStreamingThread  # deprecated

class Axis(rospy.SubscribeListener):
    def __init__(self, hostname, username, password, width, height, frame_id, camera_info_url, use_encrypted_password,
                 camera_id=1, auto_wakeup_camera=True, resolution_name=None, compression=0, fps=24, use_color=True,
                 use_square_pixels=False):
        # True every time the video parameters have changed and the URL has to be altered.
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

        # dynamic reconfigure support
        self.video_stream_param_change_server = dynamic_reconfigure.server.Server(VideoStreamConfig, self.reconfigure_video)

        # camera info setup
        self.frame_id = frame_id
        self.camera_info_url = camera_info_url

        # generate a valid camera name based on the hostname
        self.camera_name = camera_info_manager.genCameraName(self.hostname)
        self.camera_info = camera_info_manager.CameraInfoManager(cname=self.camera_name,
                                                   url=self.camera_info_url)
        self.camera_info.loadCameraInfo() # required before getCameraInfo()

        # the thread used for streaming images
        self.streaming_thread = None

        # the publishers are started/stopped lazily in peer_subscribe/peer_unsubscribe
        self.video_publisher = rospy.Publisher("image_raw/compressed", CompressedImage, self, queue_size=100)
        self.camera_info_publisher = rospy.Publisher("camera_info", CameraInfo, self, queue_size=100)

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

    def reconfigure_video(self, config, level):
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
        try:
            setter(config[field])
        except ValueError:
            config[field] = getattr(self, field)

    def set_resolution(self, resolution_name):
        if isinstance(resolution_name, basestring) and (
                self.resolution is None or resolution_name != self.resolution.name):
            self.resolution = self._get_resolution_for_name(resolution_name)
            self.video_params_changed = True
            # deprecated values
            self.width = self.resolution.get_resolution(self.use_square_pixels)[0]
            self.height = self.resolution.get_resolution(self.use_square_pixels)[1]

    def _get_resolution_for_name(self, resolution_name):
        if resolution_name not in self.allowed_resolutions:
            raise ValueError("%s is not a valid valid resolution." % resolution_name)

        return self.allowed_resolutions[resolution_name]

    def find_resolution_by_size(self, width, height):
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
        config_resolutions = self._read_resolutions_from_config()
        camera_resolutions = self._get_resolutions_supported_by_camera()

        return dict((k, v) for k, v in config_resolutions.iteritems() if k in camera_resolutions)

    @staticmethod
    def _read_resolutions_from_config():

        video_stream_parameters = VideoStreamConfig.config_description['parameters']
        resolutions = dict()

        resolution_size_parser = re.compile(r'\((?P<width>[0-9]+)x(?P<height>[0-9]+)\)')

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
        try:
            return self.api.parse_list_parameter_value(self.api.get_parameter("root.Properties.Image.Resolution"))
        except (IOError, ValueError):
            rospy.logwarn("Could not determine resolutions supported by the camera. Asssuming only CIF.")
            return ["CIF"]

    def set_compression(self, compression):
        if compression != self.compression:
            self.compression = self.sanitize_compression(compression)
            self.video_params_changed = True

    @staticmethod
    def sanitize_compression(compression):
        compression = int(compression)
        if not (0 <= compression <= 100):
            raise ValueError("%s is not a valid value for compression." % str(compression))

        return compression

    def set_fps(self, fps):
        if fps != self.fps:
            self.fps = self.sanitize_fps(fps)
            self.video_params_changed = True

    @staticmethod
    def sanitize_fps(fps):
        fps = int(fps)
        if not (1 <= fps <= 30):
            raise ValueError("%s is not a valid value for FPS." % str(fps))

        return fps

    def set_use_color(self, use_color):
        if use_color != self.use_color:
            self.use_color = self.sanitize_bool(use_color, "use_color")
            self.video_params_changed = True

    def set_use_square_pixels(self, use_square_pixels):
        if use_square_pixels != self.use_square_pixels:
            self.use_square_pixels = self.sanitize_bool(use_square_pixels, "use_square_pixels")
            self.video_params_changed = True

    @staticmethod
    def sanitize_bool(value, field_name):

        if value not in (True, False, "1", "0", 1, 0):
            raise ValueError("%s is not a valid value for %s." % (str(value), field_name))

        # bool("0") returns True because it is a nonempty string
        if value == "0":
            return False

        return bool(value)


class CIFVideoResolution(object):
    def __init__(self, name, width, height):
        super(CIFVideoResolution, self).__init__()

        self.name = name
        self.width = int(width)
        self.height = int(height)

        self.square_pixel_conversion_ratio_width = 12.0 / 11.0
        self.square_pixel_conversion_ratio_height = 1

    def __repr__(self):
        return "%s (%dx%d)" % (self.name, self.width, self.height)

    def get_resolution(self, use_square_pixels=False):
        width = self.width
        height = self.height

        if use_square_pixels:
            width = int(math.ceil(self.square_pixel_conversion_ratio_width * self.width))
            height = int(math.ceil(self.square_pixel_conversion_ratio_height * self.height))

        return width, height


def main():
    rospy.init_node("axis_driver")

    arg_defaults = {
        'hostname': '192.168.0.90',       # default IP address
        'username': 'root',               # default login name
        'password': '',
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
    '''Look up parameters starting in the driver's private parameter space, but
    also searching outer namespaces.  Defining them in a higher namespace allows
    the axis_ptz.py script to share parameters with the driver.'''
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
