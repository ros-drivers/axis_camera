#!/usr/bin/env python
#
# Axis camera image driver. Based on:
# https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/sandbox/axis_camera
# /axis.py
#
# Communication with the camera is done using the Axis VAPIX API described at
# http://www.axis.com/global/en/support/developer-support/vapix
#

import threading
import urllib2
import math
import re

import rospy 
from sensor_msgs.msg import CompressedImage, CameraInfo
import camera_info_manager
import dynamic_reconfigure.server
from axis_camera.cfg import PTZConfig, VideoStreamConfig


class StreamThread(threading.Thread):
    def __init__(self, axis):
        threading.Thread.__init__(self)
        self.axis = axis
        self.daemon = True
        self.timeoutSeconds = 2.5
        self.is_paused = True

    def run(self):
        self.resume()

        while not rospy.is_shutdown():
            self.stream()

    def resume(self):
        self.is_paused = False

    def pause(self):
        self.is_paused = True

    def stream(self):
        while not rospy.is_shutdown():
            self.authenticate()
            self.publish_frames_until_error()

            # if stream stays intact we shouldn't get to this
            if not rospy.is_shutdown():
                rospy.logerr("Video stream error. Trying again in 2 seconds")
                rospy.sleep(2)

    def get_video_url(self):
        # url = 'http://%s/mjpg/video.mjpg?fps=%d&resolution=%dx%d&compression=%d' is also possible
        url = 'http://%s/axis-cgi/mjpg/video.cgi?fps=%d&resolution=%s&compression=%d&color=%d&squarepixel=%d' % (
            self.axis.hostname, self.axis.fps, self.axis.resolution.name, self.axis.compression,
            1 if self.axis.use_color else 0,
            1 if self.axis.use_square_pixels else 0
        )

        return url

    def get_wakeup_url(self):
        url = 'http://%s/axis-cgi/com/ptz.cgi?camera=1&autofocus=on' % self.axis.hostname  # TODO check the API
        return url

    def wakeup_camera(self):
        """
        Wake up the camera from standby.
        :return: If the wakeup succeeded.
        :rtype: bool
        """
        rospy.logdebug("Trying to wake up the camera.")

        wakeup_command = self.get_wakeup_url()
        wakeup_stream = self.open_url(wakeup_command, valid_statuses=[200, 204])  # 204 = No Content

        if wakeup_stream is None:
            rospy.logwarn("Cannot get camera stream, and also cannot wake up the camera.")
            return False

        # if the wakeup succeeded, give it a while to initialize and then proceeed further
        rospy.loginfo("Camera wakeup succeeded, now waiting for it to initialize.")
        rospy.sleep(5)
        return True

    def authenticate(self):
        """only try to authenticate if user/pass configured.  I have not
        used this method (yet)."""
        if self.axis.password != '' and self.axis.username != '':
            # create a password manager
            password_mgr = urllib2.HTTPPasswordMgrWithDefaultRealm()

            # Add the username and password, use default realm.
            top_level_url = "http://" + self.axis.hostname
            password_mgr.add_password(None, top_level_url, self.axis.username, self.axis.password)
            if self.axis.use_encrypted_password:
                handler = urllib2.HTTPDigestAuthHandler(password_mgr)
            else:
                handler = urllib2.HTTPBasicAuthHandler(password_mgr)

            # create "opener" (OpenerDirector instance)
            opener = urllib2.build_opener(handler)

            # ...and install it globally so it can be used with urlopen.
            urllib2.install_opener(opener)
    
    def open_url(self, url, valid_statuses=None):
        """Open connection to Axis camera using http"""
        try:
            rospy.logdebug('Opening ' + url)
            rospy.logdebug('Camera settings are: ' + str(self.axis))
            stream = urllib2.urlopen(url, timeout=self.timeoutSeconds)
            if stream is not None and (valid_statuses is None or stream.getcode() in valid_statuses):
                return stream
            else:
                return None
        except urllib2.URLError, e:
            rospy.logwarn('Error opening URL %s' % url + 'Possible timeout.  Looping until camera appears')
            return None

    def publish_frames_until_error(self):
        """Continuous loop to publish images"""
        while not rospy.is_shutdown():
            url = self.get_video_url()
            stream = self.open_url(url)

            if stream is None:
                if self.axis.auto_wakeup_camera:
                    wakeup_succeeded = self.wakeup_camera()
                    if not wakeup_succeeded:
                        return
                else:
                    return

            rate = rospy.Rate(self.axis.fps)
            self.axis.video_params_changed = False

            while not rospy.is_shutdown() and not self.axis.video_params_changed and not self.is_paused:
                try:
                    found_boundary = self.find_boundary_in_stream(stream)
                    if not found_boundary:
                        # end of stream means we need to launch everything from scratch
                        return

                    (header, image, timestamp) = self.read_image_from_stream(stream)

                    if image is not None:
                        self.publish_image(header, image, timestamp)
                        self.publish_camera_info(header, image, timestamp)
                    else:
                        rospy.logwarn("Retrieving image from Axis camera failed.")
                except urllib2.URLError:
                    rospy.loginfo('Timed out while trying to get message.')
                    return

                # read images only on the requested FPS frequency
                rate.sleep()

            if self.axis.video_params_changed:
                rospy.logdebug("Video parameters changed, reconnecting the video stream with the new ones.")
            else:
                while not rospy.is_shutdown() and self.is_paused:
                    rospy.sleep(1)  # wait until someone unpauses us

    @staticmethod
    def find_boundary_in_stream(stream):
        """The string "--myboundary" is used to denote the start of an image in
        Axis cameras. Returns False if end of stream was reached."""
        while not rospy.is_shutdown():
            line = stream.readline()
            if line is None:
                # end of stream
                return False
            if line == '--myboundary\r\n':
                return True

    def read_image_from_stream(self, stream):
        """Get the image header and image itself"""
        header = self.get_image_header_from_stream(stream)

        image = None
        if header['Content-Length'] > 0:
            content_length = int(header['Content-Length'])
            image = self.get_image_data_from_stream(stream, content_length)

        return header, image, rospy.Time.now()

    @staticmethod
    def get_image_header_from_stream(stream):
        header = {}
        while not rospy.is_shutdown():
            line = stream.readline()
            if line == "\r\n":
                break
            line = line.strip()
            parts = line.split(": ", 1)

            if len(parts) != 2:
                rospy.logwarn('Problem encountered with image header.  Setting content_length to zero')
                header['Content-Length'] = 0  # set content_length to zero if there is a problem reading header
            else:
                try:
                    header[parts[0]] = parts[1]
                except urllib2.URLError:
                    rospy.logwarn('Problem encountered with image header.  Setting content_length to zero')
                    header['Content-Length'] = 0 # set content_length to zero if there is a problem reading header

        return header

    @staticmethod
    def get_image_data_from_stream(stream, num_bytes):
        """Get the binary image data itself (ie. without header)"""
        image = stream.read(num_bytes)
        stream.readline()  # Read terminating \r\n and do nothing with it
        return image

    def publish_image(self, header, image, timestamp):
        """Publish jpeg image as a ROS message"""
        msg = CompressedImage()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.axis.frame_id
        msg.format = "jpeg"
        msg.data = image
        self.axis.video_publisher.publish(msg)

    def publish_camera_info(self, header, image, timestamp):
        """Publish camera info manager message"""
        msg = self.axis.camera_info.getCameraInfo()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.axis.frame_id
        msg.width = self.axis.width
        msg.height = self.axis.height
        self.axis.camera_info_publisher.publish(msg)


class Axis(rospy.SubscribeListener):
    def __init__(self, hostname, username, password, width, height, frame_id, camera_info_url, use_encrypted_password,
                 auto_wakeup_camera=True, resolution_name=None, compression=0, fps=24, use_color=True, use_square_pixels=False):
        # True every time the video parameters have changed and the URL has to be altered.
        self.video_params_changed = False

        self.hostname = hostname
        self.username = username
        self.password = password

        self.allowed_resolutions = self._get_allowed_resolutions()
        rospy.logwarn(self.allowed_resolutions)

        # Sometimes the camera falls into power saving mode and stops streaming.
        # This setting allows the script to try to wake up the camera.
        self.auto_wakeup_camera = auto_wakeup_camera

        self.width = None
        self.height = None
        self.resolution = None
        self.compression = None
        self.fps = None
        self.use_color = None
        self.use_square_pixels = None

        self.set_compression(compression)
        self.set_fps(fps)
        self.set_use_color(use_color)
        self.set_use_square_pixels(use_square_pixels)

        if resolution_name is not None:
            self.set_resolution(resolution_name)
        else:
            resolution = self.find_resolution_by_size(width, height)
            self.set_resolution(resolution.name)

        self.frame_id = frame_id
        self.camera_info_url = camera_info_url
        self.use_encrypted_password = use_encrypted_password
        
        # generate a valid camera name based on the hostname
        self.camera_name = camera_info_manager.genCameraName(self.hostname)
        self.camera_info = camera_info_manager.CameraInfoManager(cname=self.camera_name,
                                                   url=self.camera_info_url)
        self.camera_info.loadCameraInfo() # required before getCameraInfo()
        self.streaming_thread = None

        # The publishers are started lazily in peer_subscribe/peer_unsubscribe
        self.video_publisher = rospy.Publisher("image_raw/compressed", CompressedImage, self, queue_size=100)
        self.camera_info_publisher = rospy.Publisher("camera_info", CameraInfo, self, queue_size=100)

        self.video_stream_param_change_server = dynamic_reconfigure.server.Server(VideoStreamConfig, self.reconfigure_video)

    def __str__(self):
        """Return string representation."""
        return(self.hostname + ',' + self.username + ',' + self.password +
                       '(' + str(self.width) + 'x' + str(self.height) + ')')

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        """Lazy-start the image-publisher."""
        if self.streaming_thread is None:
            self.streaming_thread = StreamThread(self)
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
        url = "http://%s/axis-cgi/view/param.cgi?action=list&group=root.Properties.Image.Resolution" % self.hostname
        resolution_stream = self.open_url(url)

        if resolution_stream is not None:
            line = resolution_stream.readline()
            values = line.strip().split("=")
            resolutions = values[1].split(",")
            return resolutions

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

    def set_resolution(self, resolution_name):
        if isinstance(resolution_name, basestring) and (self.resolution is None or resolution_name != self.resolution.name):
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

    def open_url(self, url, valid_statuses=None):
        """Open connection to Axis camera using http"""
        try:
            rospy.logdebug('Opening ' + url)
            stream = urllib2.urlopen(url, timeout=2.5)
            if stream is not None and (valid_statuses is None or stream.getcode() in valid_statuses):
                return stream
            else:
                return None
        except urllib2.URLError, e:
            rospy.logwarn('Error opening URL %s' % url + 'Possible timeout.  Looping until camera appears')
            return None


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
        'auto_wakeup_camera': True,
        'width': 704,
        'height': 576,
        'compression': 0,
        'fps': 24,
        'use_color': True,
        'use_square_pixels': False,
        'frame_id': 'axis_camera',
        'camera_info_url': '',
        'use_encrypted_password': False}
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
