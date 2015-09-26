import threading
from math import radians

import rospy
from sensor_msgs.msg import JointState
from dynamic_reconfigure.msg import Config, IntParameter, DoubleParameter

from axis_camera.msg import Axis, PTZ


class PositionStreamingThread(threading.Thread):
    """
    This class handles publication of the positional state of the camera to a ROS message and to joint_states
    (and using robot_state_publisher also to TF).

    The thread doesn't support pausing, because it doesn't make sense if we want TF data.
    """

    def __init__(self, axis, api):
        """
        Create the thread.
        :param axis: The parameter source.
        :type axis: AxisPTZ
        :param api: The VAPIX API instance that allows the thread to access positional data.
        :type api: VAPIX
        """
        threading.Thread.__init__(self)

        self.axis = axis
        self.api = api

        # Permit program to exit even if threads are still running by flagging
        # thread as a daemon:
        self.daemon = True

        # BACKWARDS COMPATIBILITY LAYER
        self.cameraPosition = None  # deprecated
        self.msg = Axis()  # deprecated

    def run(self):
        """Run the thread."""
        rate = rospy.Rate(self.axis.state_publishing_frequency)

        state_publisher = rospy.Publisher("camera/ptz", PTZ, queue_size=100)
        joint_states_publisher = rospy.Publisher("camera/joint_states", JointState, queue_size=100)
        parameter_updates_publisher = rospy.Publisher("axis_ptz/parameter_updates", Config, queue_size=100)

        while not rospy.is_shutdown():
            # publish position forever
            try:
                # get camera position from API
                camera_position = self.api.get_camera_position()  # we use deprecated param values
                message_time = rospy.Time.now()

                # Create and publish the PTZ message
                message = self._create_camera_position_message(camera_position, message_time)
                state_publisher.publish(message)

                # Create and publish JointState
                joint_states_message = self._create_camera_joint_states_message(camera_position, message_time)
                joint_states_publisher.publish(joint_states_message)

                # Publish the parameter updates (because of backwards compatibility)
                parameter_updates_message = self._create_parameter_updates_message(camera_position)
                parameter_updates_publisher.publish(parameter_updates_message)

                # set the parameters (because of backwards compatibility)
                rospy.set_param("axis_ptz/pan", camera_position['pan'])
                rospy.set_param("axis_ptz/tilt", camera_position['tilt'])
                rospy.set_param("axis_ptz/zoom", camera_position['zoom'])

                # Create and publish the deprecated Axis message
                self.cameraPosition = camera_position  # backwards compatibility
                self.publishCameraState()  # backwards compatibility
            except (IOError, ValueError, RuntimeError) as e:
                rospy.logwarn("Could not determine current camera position. Waiting 1 s. Cause: %s" % repr(e.args))
                rospy.sleep(1)

            rate.sleep()

    def _create_camera_position_message(self, camera_position, timestamp):
        """
        Convert the camera_position dictionary to a PTZ message.
        :param camera_position: The camera position. Should contain keys 'pan', 'tilt', and probably also 'zoom'.
        :type camera_position: dict
        :param timestamp: The time we relate the camera position to.
        :type timestamp: rospy.Time
        :return: The PTZ message.
        :rtype: PTZ
        """
        message = PTZ()

        message.header.stamp = timestamp
        message.header.frame_id = self.axis.frame_id

        message.pan = camera_position['pan']
        message.tilt = camera_position['tilt']
        if 'zoom' in camera_position:
            message.zoom = camera_position['zoom']

        if self.axis.flip:
            self._correct_flipped_pan_tilt_in_message(message)

        return message

    def _create_camera_joint_states_message(self, camera_position, timestamp):
        """
        Convert the camera_position dictionary to a JointState message.
        :param camera_position: The camera position. Should contain keys 'pan', 'tilt', and probably also 'zoom'.
        :type camera_position: dict
        :param timestamp: The time we relate the camera position to.
        :type timestamp: rospy.Time
        :return: The JointState message.
        :rtype: JointState
        """
        message = JointState()

        message.header.stamp = timestamp
        message.header.frame_id = self.axis.frame_id

        message.name = ["axis_pan_j", "axis_tilt_j"]
        message.position = [radians(camera_position['pan']), radians(camera_position['tilt'])]

        # TODO message.velocity???
        # TODO flipping?

        return message

    def _create_parameter_updates_message(self, camera_position):
        """
        Create a parameter_updates message to update the deprecated dynamic_reconigurable pan, tilt and zoom params.
        :param camera_position: The camera position. Should contain keys 'pan', 'tilt', and probably also 'zoom'.
        :type camera_position: dict
        :return: The Config message.
        :rtype: Config
        """
        message = Config()

        message.doubles.append(DoubleParameter(name="pan", value=camera_position['pan']))
        message.doubles.append(DoubleParameter(name="tilt", value=camera_position['tilt']))
        message.ints.append(IntParameter(name="zoom", value=camera_position['zoom']))

        return message

    @staticmethod
    def _correct_flipped_pan_tilt_in_message(message):
        """
        If flipping the image is required, do the flipping on a PTZ message fields.
        :param message: A PTZ or Axis message.
        :type message: PTZ|Axis
        """
        message.pan = 180 - message.pan
        if message.pan > 180:
            message.pan -= 360
        elif message.pan < -180:
            message.pan += 360
        message.tilt = -message.tilt

    # BACKWARDS COMPATIBILITY LAYER

    def queryCameraPosition(self):  # deprecated
        pass  # is done in the run method

    def publishCameraState(self):  # deprecated
        if self.cameraPosition is not None:
            self.msg.pan = float(self.cameraPosition['pan'])
            if self.axis.flip:
                self.adjustForFlippedOrientation()
            self.msg.tilt = float(self.cameraPosition['tilt'])
            self.msg.zoom = float(self.cameraPosition['zoom'])
            self.msg.iris = 0.0
            if 'iris' in self.cameraPosition:
                self.msg.iris = float(self.cameraPosition['iris'])
            self.msg.focus = 0.0
            if 'focus' in self.cameraPosition:
                self.msg.focus = float(self.cameraPosition['focus'])
            if 'autofocus' in self.cameraPosition:
                self.msg.autofocus = (self.cameraPosition['autofocus'] == 'on')
            self.axis.pub.publish(self.msg)

    def adjustForFlippedOrientation(self):  # deprecated
        self._correct_flipped_pan_tilt_in_message(self.msg)
