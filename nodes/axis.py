#!/usr/bin/env python
#
# Axis camera image driver. Based on:
# https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/sandbox/axis_camera
# /axis.py
#

import threading
import urllib2

import rospy 
from sensor_msgs.msg import CompressedImage, CameraInfo
import camera_info_manager
import dynamic_reconfigure.server
from axis_camera.cfg import PTZConfig


class StreamThread(threading.Thread):
    def __init__(self, axis):
        threading.Thread.__init__(self)
        self.axis = axis
        self.daemon = True
        self.timeoutSeconds = 2.5
        self.should_run = False

    def run(self):
        self.resume()

        while not rospy.is_shutdown():
            self.stream()

    def resume(self):
        self.should_run = True

    def pause(self):
        self.should_run = False

    def stream(self):
        while not rospy.is_shutdown():
            self.authenticate()
            self.publish_frames_until_error()

            # if stream stays intact we shouldn't get to this
            if not rospy.is_shutdown():
                rospy.logerr("Video stream error. Trying again in 2 seconds")
                rospy.sleep(2)

    def get_video_url(self):
        url = 'http://%s/mjpg/video.mjpg?req_fps=%d&resolution=%dx%d&compression=%d' % (
            self.axis.hostname, self.axis.fps, self.axis.width, self.axis.height, self.axis.compression
        )

        return url

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
    
    def open_url(self, url):
        """Open connection to Axis camera using http"""
        try:
            rospy.logdebug('Opening ' + url)
            rospy.logdebug('Camera settings are: ' + str(self.axis))
            return urllib2.urlopen(url, timeout=self.timeoutSeconds)
        except urllib2.URLError, e:
            rospy.logwarn('Error opening URL %s' % url + 'Possible timeout.  Looping until camera appears')
            return None

    def publish_frames_until_error(self):
        """Continuous loop to publish images"""
        while not rospy.is_shutdown():
            url = self.get_video_url()
            stream = self.open_url(url)

            if stream is None:
                return

            rate = rospy.Rate(self.axis.fps)
            self.axis.video_params_changed = False

            while not rospy.is_shutdown() and not self.axis.video_params_changed:
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

            rospy.logdebug("Video parameters changed, reconnecting the video stream with the new ones.")

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
    def __init__(self, hostname, username, password, width, height, compression,
                 fps, frame_id, camera_info_url, use_encrypted_password):
        # True every time the video parameters have changed and the URL has to be altered.
        self.video_params_changed = False

        self.allowed_resolutions = set([(704, 576), (352, 288), (176, 144)])

        self.hostname = hostname
        self.username = username
        self.password = password

        self.width = None
        self.height = None
        self.compression = None
        self.fps = None
        self.set_resolution(width, height)
        self.set_compression(compression)
        self.set_fps(fps)

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
        self.dynamic_reconfigure_server = dynamic_reconfigure.server.Server(PTZConfig, self.reconfigure)

    def __str__(self):
        """Return string representation."""
        return(self.hostname + ',' + self.username + ',' + self.password +
                       '(' + str(self.width) + 'x' + str(self.height) + ')')

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        """Lazy-start the image-publisher."""
        if self.streaming_thread is None:
            self.streaming_thread = StreamThread(self)
            self.streaming_thread.start()

    def peer_unsubscribe(self, topic_name, num_peers):
        """Lazy-stop the image-publisher when nobody is interested"""
        if num_peers == 0:
            self.streaming_thread.pause()

    def reconfigure(self, config, level):
        try:
            self.set_compression(config.compression)
        except ValueError:
            config.compression = self.compression

        try:
            self.set_fps(config.fps)
        except ValueError:
            config.fps = self.fps

        try:
            resolution_parts = config.resolution.split("x", 2)
            if not len(resolution_parts) == 2:
                config.resolution = "%dx%d" % (self.width, self.height)
            else:
                self.set_resolution(resolution_parts[0], resolution_parts[1])
        except ValueError:
            config.resolution = "%dx%d" % (self.width, self.height)

        return config

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

    def set_resolution(self, width, height):
        if width != self.width or height != self.height:
            (self.width, self.height) = self.sanitize_resolution(width, height)
            self.video_params_changed = True

    def sanitize_resolution(self, width, height):
        width = int(width)
        height = int(height)
        if not (width, height) in self.allowed_resolutions:
            raise ValueError("%sx%s is not a valid value resolution." % (str(width), str(height)))

        return width, height


def main():
    rospy.init_node("axis_driver")

    arg_defaults = {
        'hostname': '192.168.0.90',       # default IP address
        'username': 'root',               # default login name
        'password': '',
        'width': 704,
        'height': 576,
        'compression': 0,
        'fps': 24,
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
