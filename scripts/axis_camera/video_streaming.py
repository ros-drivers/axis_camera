from contextlib import closing
import threading
import rospy
from sensor_msgs.msg import CompressedImage


class ImageStreamingThread(threading.Thread):
    def __init__(self, axis):
        threading.Thread.__init__(self)
        self.axis = axis
        self.daemon = True
        self.timeoutSeconds = 2.5
        self.is_paused = True

        self.fp = None  # deprecated

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
            self.publish_frames_until_error()

            # if stream stays intact we shouldn't get to this
            if not rospy.is_shutdown():
                rospy.logerr("Video stream error. Trying again in 2 seconds")
                rospy.sleep(2)

    def wakeup_camera(self):
        """
        Wake up the camera from standby.
        :return: If the wakeup succeeded.
        :rtype: bool
        :raises: IOException
        """
        rospy.logdebug("Trying to wake up the camera.")

        self.axis.api.wakeup_camera(self.axis.hostname, self.axis.camera_id)

        # if the wakeup succeeded, give it a while to initialize and then proceeed further
        rospy.loginfo("Camera wakeup succeeded, now waiting for it to initialize.")
        rospy.sleep(5)
        return True

    def publish_frames_until_error(self):
        """
        Continuous loop to publish images. Stops if an error is occured while reading the stream.

        Should also account for pausing/resuming (stop/start reading the stream), and video parameters changes.
        These should not make this function exit, it should instead reconnect automatically.
        """
        while not rospy.is_shutdown():  # publish until stopped

            # if we are paused, wait for a resume before connecting to the stream
            while not rospy.is_shutdown() and (self.is_paused or not self.axis.api.is_video_ok()):
                rospy.sleep(1)

            try:
                # now try to open the video stream;
                with closing(self.axis.api.get_video_stream(self.axis.fps, self.axis.resolution.name, self.axis.compression,
                                                            self.axis.use_color, self.axis.use_square_pixels)) as stream:

                    self.fp = stream  # backwards compatibility

                    rate = rospy.Rate(self.axis.fps)
                    self.axis.video_params_changed = False

                    # publish images from the stream until i) someone pauses us, or ii) video parameters changed
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
                                # probably a temporary error, don't return
                                rospy.logwarn("Retrieving image from Axis camera failed.")
                        except IOError:
                            rospy.loginfo('Timed out while trying to get message.')
                            # error occured, we should therefore return
                            return

                        # read images only on the requested FPS frequency
                        rate.sleep()

            except IOError:
                # could not open the stream, the camera is probably in standby mode?
                if self.axis.auto_wakeup_camera:
                    wakeup_succeeded = self.wakeup_camera()
                    if not wakeup_succeeded:
                        # if we could not wake up the camera, it is an error and we should return
                        return
                    else:
                        continue
                else:
                    # if we cannot open the stream and auto-wakeup is off, we have no other possibility than to report
                    # error
                    return

            # the image-publishing loop has been interrupted, so reconnect to the stream in the next
            # outer-while iteration

            if self.axis.video_params_changed:
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
                except IOError:
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

    # BACKWARDS COMPATIBILITY LAYER
    def authenticate(self):  # is already done when connecting to VAPIX
        pass

    def publishFramesContinuously(self):
        self.publish_frames_until_error()

    def findBoundary(self):
        self.find_boundary_in_stream(self.fp)

    def getImage(self):
        self.getHeader()
        self.getImageData()

    def getHeader(self):
        self.header = self.get_image_header_from_stream(self.fp)
        self.content_length = int(self.header['Content-Length'])

    def getImageData(self):
        if self.content_length > 0:
            self.img = self.get_image_data_from_stream(self.fp, self.content_length)

    def publishMsg(self):
        self.publish_image(self.header, self.img, rospy.Time.now())

    def publishCameraInfoMsg(self):
        self.publish_camera_info(self.header, self.img, rospy.Time.now())
