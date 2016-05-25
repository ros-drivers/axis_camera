from contextlib import closing
import threading

import rospy
from sensor_msgs.msg import CompressedImage
from axis_camera.vapix import VAPIX


class ImageStreamingThread(threading.Thread):
    """
    A thread that handles streaming video from the camera.

    It supports pausing the video stream (e.g. when there is no receiver of the video data), which should lower the
    usage of communications link with the camera.
    """

    def __init__(self, axis):
        """Create the video streaming thread (in a paused state).

        :param axis: The parameter specifying class.
        :type axis: :py:class:`axis.Axis`
        """
        threading.Thread.__init__(self)
        self.daemon = True  # allow exiting the main program even if this thread is still running

        # TODO refactor so that we pass the parameters directly and not using self.axis
        self._axis = axis
        self._is_paused = True

        # BACKWARDS COMPATIBILITY LAYER
        self.fp = None  # deprecated
        self.header = None  # deprecated
        self.img = None  # deprecated
        self.content_length = None  # deprecated

    def run(self):
        """
        This method is executed when the thread execution should start (for the first time only, do not call more times)
        """
        self.resume()

        while not rospy.is_shutdown():
            self._stream()

    def resume(self):
        """Start streaming if the video stream was paused."""
        self._is_paused = False

    def pause(self):
        """Pause streaming if the video stream was streaming."""
        self._is_paused = True

    def is_paused(self):
        """Return if the streaming thread is paused or not.

        :return bool: The paused status.
        """
        return self._is_paused

    def _stream(self):
        """Stream the video forever, taking into account the paused state.

        Try to stream again after a streaming error.
        """
        while not rospy.is_shutdown():
            self._publish_frames_until_error()

            # if stream stays intact we shouldn't get to this
            if not rospy.is_shutdown():
                rospy.logerr("Video stream error. Trying again in 2 seconds")
                rospy.sleep(2)

    def _wakeup_camera(self):
        """Wake up the camera from standby.

        This call is blocking and waits for some time so that the initialization should be more or less finished when
        it finishes.

        :return: If the wakeup succeeded.
        :rtype: :py:class:`bool`
        """
        rospy.logdebug("Trying to wake up the camera.")

        try:
            self._axis._api._wakeup_camera(self._axis.hostname, self._axis._camera_id)

            # if the wakeup succeeded, give the camera a while to initialize and then proceed further
            rospy.loginfo("Camera wakeup succeeded, now waiting for the camera to initialize.")
            rospy.sleep(5)
            return True
        except IOError, e:
            rospy.logerr("Exception thrown when waking up the camera. Cause: %r" % e)
            return False

    def _publish_frames_until_error(self):
        """Continuous loop to publish images. Stops if an error occurs while reading the stream.

        Should also account for pausing/resuming (stop/start reading the stream), and video parameters changes.
        These should not make this function exit, it should instead reconnect automatically.
        """
        while not rospy.is_shutdown():  # publish until an error occurs (then, a return is called to end this loop)

            # if we are paused, wait for a resume before connecting to the stream
            while not rospy.is_shutdown() and (self._is_paused or not self._is_video_ok()):
                rospy.sleep(1)

            try:  # now try to open the video stream;
                with closing(
                        self._axis._api.get_video_stream(self._axis._fps, self._axis._resolution.name, self._axis._compression,
                                                         self._axis._use_color, self._axis._use_square_pixels)
                ) as stream:

                    self.fp = stream  # backwards compatibility

                    # this is the rate at which we wake up to read new data; there is no need for it to happen faster
                    # than at the requested FPS
                    rate = rospy.Rate(self._axis._fps)

                    # we've just crerated a video stream with the most recently requested parameters, so clear the dirty
                    # flag
                    self._axis.video_params_changed = False

                    # publish images from the stream until i) someone pauses us, or ii) video parameters changed
                    # when one of these events happens, we take a next iteration of the outermost loop, in which there
                    # is a wait in case of pausing, and a new video stream connection is created with most recent
                    # parameters
                    while not rospy.is_shutdown() and not self._axis.video_params_changed and not self._is_paused:
                        try:
                            (header, image) = VAPIX.read_next_image_from_video_stream(stream)
                            if image is not None:
                                timestamp = rospy.Time.now()
                                self._publish_image(header, image, timestamp)
                                self._publish_camera_info(header, image, timestamp)
                            else:
                                # probably a temporary error, don't return
                                rospy.logwarn("Retrieving image from Axis camera failed.")
                        except IOError, e:
                            # error occured, we should therefore return
                            rospy.loginfo('Error reading from the video stream. Cause: %r' % e)
                            return

                        # read images only on the requested FPS frequency
                        rate.sleep()

            except IOError, e:
                # could not open the stream, the camera is probably in standby mode?
                if self._axis.auto_wakeup_camera:
                    wakeup_succeeded = self._wakeup_camera()
                    if not wakeup_succeeded:
                        # if we could not wake up the camera, it is an error and we should return
                        rospy.logerr("%r" % e)
                        return
                    else:
                        continue
                else:
                    # if we cannot open the stream and auto-wakeup is off, we have no other possibility than to report
                    # error
                    rospy.logerr("%r" % e)
                    return

            # the image-publishing loop has been interrupted, so reconnect to the stream in the next
            # outer-while iteration

            if self._axis.video_params_changed:
                rospy.logdebug("Video parameters changed, reconnecting the video stream with the new ones.")

    def _is_video_ok(self):
        """Return True if the video stream is in good condition.

        :return: True if the video stream is in good condition.
        :rtype: :py:class:`bool`
        """
        try:
            return self._axis._api.is_video_ok()
        except IOError, e:
            rospy.logerr(repr(e))
            return False

    def _publish_image(self, header, image, timestamp):
        """Publish JPG image as a ROS message.

        :param header: The HTTP-like header read from the stream.
        :type header: dict
        :param image: The video frame in JPG.
        :type image: :py:obj:`basestring`
        :param timestamp: The time when the frame was captured (or its best estimation).
        :type timestamp: :py:class:`rospy.Time`
        """
        msg = CompressedImage()
        msg.header.stamp = timestamp
        msg.header.frame_id = self._axis._frame_id
        msg.format = "jpeg"
        msg.data = image
        self._axis._video_publisher.publish(msg)

    def _publish_camera_info(self, header, image, timestamp):
        """Publish camera info corresponding to the given image.

        :param header: The HTTP-like header read from the stream.
        :type header: dict
        :param image: The video frame in JPG.
        :type image: :py:obj:`basestring`
        :param timestamp: The time when the frame was captured (or its best estimation).
        :type timestamp: :py:class:`rospy.Time`
        """
        msg = self._axis._camera_info.getCameraInfo()
        msg.header.stamp = timestamp
        msg.header.frame_id = self._axis._frame_id
        msg.width = self._axis._width
        msg.height = self._axis._height
        self._axis._camera_info_publisher.publish(msg)

    # BACKWARDS COMPATIBILITY LAYER
    def authenticate(self):  # is already done when connecting to VAPIX
        """Deprecated."""
        pass

    def publishFramesContinuously(self):
        """Deprecated."""
        self._publish_frames_until_error()

    def findBoundary(self):
        """Deprecated."""
        VAPIX._find_boundary_in_stream(self.fp)

    def getImage(self):
        """Deprecated."""
        self.getHeader()
        self.getImageData()

    def getHeader(self):
        """Deprecated."""
        self.header = VAPIX._get_image_header_from_stream(self.fp)
        self.content_length = int(self.header['Content-Length'])

    def getImageData(self):
        """Deprecated."""
        if self.content_length > 0:
            self.img = VAPIX._get_image_data_from_stream(self.fp, self.content_length)

    def publishMsg(self):
        """Deprecated."""
        self._publish_image(self.header, self.img, rospy.Time.now())

    def publishCameraInfoMsg(self):
        """Deprecated."""
        self._publish_camera_info(self.header, self.img, rospy.Time.now())
