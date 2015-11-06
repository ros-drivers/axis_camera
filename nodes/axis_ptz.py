#!/usr/bin/env python
#
# Basic VAPIX PTZ node, based on documentation here:
#   http://www.axis.com/global/en/support/developer-support/vapix

import threading

import rospy
from axis_camera.msg import Axis
from std_msgs.msg import Bool
from dynamic_reconfigure.server import Server
from axis_camera.cfg import PTZConfig

from axis_camera.vapix import VAPIX
from axis_camera.position_streaming import PositionStreamingThread
from axis_camera.camera_control import AxisCameraController

StateThread = PositionStreamingThread  # deprecated


class AxisPTZ:
    """This class is a node to manage the PTZ functions of an Axis PTZ camera. The most of its work is done by
     :py:class:`AxisCameraController <axis_camera.camera_control.AxisCameraController>` and this is just a ROS node
     envelope.
    """

    def __init__(self, hostname, username, password, flip, speed_control, frame_id="axis_camera",
                 use_encrypted_password=False, state_publishing_frequency=50, camera_id=1):
        """Initialize the PTZ driver and start publishing positional data.

        :param hostname: Hostname of the camera (without http://, can be an IP address).
        :type hostname: basestring
        :param username: If login is needed, provide a username here.
        :type username: basestring|None
        :param password: If login is needed, provide a password here.
        :type password: basestring|None
        :param flip: Whether to flip the controls (for ceiling-mounted cameras). Deprecated.
        :type flip: bool
        :param speed_control: Use speed control instead of positional. Deprecated.
        :type speed_control: bool
        :param frame_id: Id of the frame in which positinal data should be published.
        :type frame_id: basestring
        :param use_encrypted_password: Whether to use Plain HTTP Auth (False) or Digest HTTP Auth (True).
        :type use_encrypted_password: bool
        :param state_publishing_frequency: The frequency at which joint states should be published.
        :type state_publishing_frequency: int
        :param camera_id: ID (number) of the camera. Can be 1 to 4.
        :type camera_id: int
        """

        self._hostname = hostname
        self._camera_id = camera_id
        self._frame_id = frame_id

        self._state_publishing_frequency = state_publishing_frequency

        self._executing_reconfigure = False
        self._reconfigure_mutex = threading.Lock()

        self._api = None
        # autodetect the VAPIX API and connect to it; try it forever
        while self._api is None and not rospy.is_shutdown():
            try:
                self._api = VAPIX.get_api_for_camera(hostname, username, password, camera_id, use_encrypted_password)
            except (IOError, ValueError):
                rospy.loginfo(
                    "Retrying connection to VAPIX on host %s, camera %d in 2 seconds." % (hostname, camera_id))
                rospy.sleep(2)
        if rospy.is_shutdown():
            return

        if not self._api.has_ptz():
            raise RuntimeError("Camera %d on host %s doesn't have a Pan-Tilt-Zoom unit." % (self._camera_id, self._hostname))

        # Create a controller of the camera
        self._camera_controller = AxisCameraController(self._api, self, flip_vertically=flip, flip_horizontally=flip)

        # BACKWARDS COMPATIBILITY LAYER
        self.username = username  # deprecated
        self.password = password  # deprecated
        self.flip = flip  # deprecated
        self.speedControl = speed_control  # deprecated
        self.mirror = False  # deprecated

        self.msg = None  # deprecated
        self.cmdString = ""  # deprecated

        self.pub = rospy.Publisher("state", Axis, queue_size=100)  # deprecated
        self.command_subscriber = rospy.Subscriber("cmd", Axis, self.cmd, queue_size=100)  # deprecated
        self.mirror_subscriber = rospy.Subscriber("mirror", Bool, self.mirrorCallback, queue_size=100)  # deprecated

        self.srv = Server(PTZConfig, self.callback)  # deprecated

        # Needs to be after the backwards compatibility setup
        # start the publisher thread
        self._publisher_thread = PositionStreamingThread(self, self._api)
        self.st = self._publisher_thread  # deprecated
        self._publisher_thread.start()

    # BACKWARDS COMPATIBILITY LAYER

    def cmd(self, message):
        """Deprecated."""
        self.msg = message

        self.sanitisePTZCommands()

        if self._api.has_capabilities('AbsolutePan', 'AbsoluteTilt', 'AbsoluteZoom'):
            self._camera_controller.set_ptz(message.pan, message.tilt, message.zoom)
        else:
            rospy.loginfo("Camera on host %s doesn't support PTZ control." % self._hostname)

        if self._api.has_capability('AbsoluteFocus'):
            self._camera_controller.set_focus(message.focus, set_also_autofocus=False)
        else:
            rospy.loginfo("Camera on host %s doesn't support absolute focus control." % self._hostname)

        if self._api.has_capability('AutoFocus'):
            if message.focus != self._camera_controller._focus:
                self._camera_controller.set_autofocus(False)
            else:
                self._camera_controller.set_autofocus(message.autofocus)
        else:
            rospy.loginfo("Camera on host %s doesn't support autofocus." % self._hostname)

        if self._api.has_capability('AutoIris'):
            self._camera_controller.set_autoiris(True)
        else:
            rospy.loginfo("Camera on host %s doesn't support autoiris." % self._hostname)

        # there is no capability for brightness
        self._camera_controller.set_brightness(message.brightness)

    def adjustForFlippedOrientation(self):
        """Deprecated."""
        self.msg.tilt = -self.msg.tilt
        if self.speedControl:
            self.msg.pan = -self.msg.pan
        else:
            self.msg.pan = 180.0 - self.msg.pan
    
    def sanitisePTZCommands(self):
        """Deprecated."""
        if not self.speedControl:
            self.msg.pan = self._api.ptz_limits['Pan'].absolute.crop_value(self.msg.pan)
            self.msg.tilt = self._api.ptz_limits['Tilt'].absolute.crop_value(self.msg.tilt)
            self.msg.zoom = self._api.ptz_limits['Zoom'].absolute.crop_value(self.msg.zoom)
            self.msg.focus = self._api.ptz_limits['Focus'].absolute.crop_value(self.msg.focus)
            self.msg.brightness = self._api.ptz_limits['Brightness'].absolute.crop_value(self.msg.brightness)
            self.msg.iris = self._api.ptz_limits['Iris'].absolute.crop_value(self.msg.iris)
        else:
            self.msg.pan = self._api.ptz_limits['Pan'].velocity.crop_value(self.msg.pan)
            self.msg.tilt = self._api.ptz_limits['Tilt'].velocity.crop_value(self.msg.tilt)
            self.msg.zoom = self._api.ptz_limits['Zoom'].velocity.crop_value(self.msg.zoom)
            self.msg.focus = self._api.ptz_limits['Focus'].velocity.crop_value(self.msg.focus)
            self.msg.brightness = self._api.ptz_limits['Brightness'].velocity.crop_value(self.msg.brightness)
            self.msg.iris = self._api.ptz_limits['Iris'].velocity.crop_value(self.msg.iris)

    def sanitisePan(self):
        """Deprecated."""
        if self.speedControl:
            self.msg.pan = self._api.ptz_limits['Pan'].velocity.crop_value(self.msg.pan)
        else:
            self.msg.pan = self._api.ptz_limits['Pan'].absolute.crop_value(self.msg.pan)

    def sanitiseTilt(self):
        """Deprecated."""
        if self.speedControl:
            self.msg.tilt = self._api.ptz_limits['Tilt'].velocity.crop_value(self.msg.tilt)
        else:
            self.msg.tilt = self._api.ptz_limits['Tilt'].absolute.crop_value(self.msg.tilt)

    def sanitiseZoom(self):
        """Deprecated."""
        if self.speedControl:
            self.msg.zoom = self._api.ptz_limits['Zoom'].velocity.crop_value(self.msg.zoom)
        else:
            self.msg.zoom = self._api.ptz_limits['Zoom'].absolute.crop_value(self.msg.zoom)
        
    def sanitiseFocus(self):
        """Deprecated."""
        if self.speedControl:
            self.msg.focus = self._api.ptz_limits['Focus'].velocity.crop_value(self.msg.focus)
        else:
            self.msg.focus = self._api.ptz_limits['Focus'].absolute.crop_value(self.msg.focus)
            
    def sanitiseBrightness(self):
        """Deprecated."""
        if self.speedControl:
            self.msg.brightness = self._api.ptz_limits['Brightness'].velocity.crop_value(self.msg.brightness)
        else:
            self.msg.brightness = self._api.ptz_limits['Brightness'].absolute.crop_value(self.msg.brightness)

    def sanitiseIris(self):
        """Deprecated."""
        if self.msg.iris > 0.000001:
            rospy.logwarn("Iris value is read-only.")

    def applySetpoints(self):
        """Deprecated."""
        """Apply the command to the camera using the HTTP API"""
        self._camera_controller.set_ptz(self.msg.pan, self.msg.tilt, self.msg.zoom)
        self._camera_controller.set_autofocus(self.msg.autofocus)
        if not self.msg.autofocus:
            self._camera_controller.set_focus(self.msg.focus)
        self._camera_controller.set_autoiris(True)
        self._camera_controller.set_brightness(self.msg.brightness)

    def createCmdString(self):
        """Deprecated."""
        """Created tje HTTP API string to command PTZ camera"""
        self.cmdString = '/axis-cgi/com/ptz.cgi?'
        if self.speedControl:
            self.cmdString += 'continuouspantiltmove=%d,%d&' % (int(self.msg.pan), int(self.msg.tilt))
            self.cmdString += 'continuouszoommove=%d&' % self.msg.zoom
            self.cmdString += 'continuousbrightnessmove=%d&' % self.msg.brightness
            # Note that brightness adjustment has no effect for Axis 214PTZ.
            if self.msg.autofocus:
                self.cmdString += 'autofocus=on&'
            else:
                self.cmdString += 'autofocus=off&continuousfocusmove=%d&' % self.msg.focus
            self.cmdString += 'autoiris=on'
        else: # position control:
            self.cmdString += 'pan=%f&tilt=%f&' % (self.msg.pan, self.msg.tilt)
            self.cmdString += 'zoom=%d&' % self.msg.zoom
            self.cmdString += 'brightness=%d&' % self.msg.brightness
            if self.msg.autofocus:
                self.cmdString += 'autofocus=on&'
            else:
                self.cmdString += 'autofocus=off&focus=%d&' % self.msg.focus
            self.cmdString += 'autoiris=on'

    def mirrorCallback(self, msg):
        """Deprecated."""
        '''Command the camera with speed control or position control commands'''
        self.mirror = msg.data
        self._camera_controller.mirror_horizontally = self.mirror
        
    def callback(self, config, level):
        """Deprecated."""
        #self.speedControl = config.speed_control

        if self._executing_reconfigure or (hasattr(self, '_camera_controller') and (self._camera_controller._executing_parameter_update or self._camera_controller._executing_reconfigure)):
            return config

        with self._reconfigure_mutex:
            self._executing_reconfigure = True

            # create temporary message and fill with data from dynamic reconfigure
            command = Axis()
            command.pan = config.pan
            command.tilt = config.tilt
            command.zoom = config.zoom
            command.focus = config.focus
            command.brightness = config.brightness
            command.autofocus = config.autofocus

            # check sanity and apply values
            self.cmd(command)

            # read sanitized values and update GUI
            config.pan = command.pan
            config.tilt = command.tilt
            config.zoom = command.zoom
            config.focus = self._camera_controller._focus
            config.brightness = self._camera_controller._brightness
            config.autofocus = self._camera_controller._autofocus

            self._executing_reconfigure = False

            # update GUI with sanitized values
            return config


def main():
    rospy.init_node("axis_ptz_driver")

    arg_defaults = {
        'hostname': '192.168.0.90',
        'username': None,
        'password': None,
        'flip': False,
        'speed_control': False,
        'frame_id': 'axis_camera',
        'use_encrypted_password': False,
        'state_publishing_frequency': 50,
        'camera_id': 1,
        }
    args = read_args_with_defaults(arg_defaults)

    # Start the driver
    my_ptz = AxisPTZ(**args)

    rospy.spin()


def read_args_with_defaults(arg_defaults):
    """Look up parameters starting in the driver's private parameter space, but
    also searching outer namespaces.  Defining them in a higher namespace allows
    the axis_ptz.py script to share parameters with the driver."""
    args = {}
    for name, val in arg_defaults.iteritems():
        full_name = rospy.search_param(name)
        if full_name is None:
            args[name] = val
        else:
            args[name] = rospy.get_param(full_name, val)
    # resolve frame_id with tf_prefix (unless already absolute)
    if 'frame_id' in args and args['frame_id'][0] != '/':  # not absolute?
        tf_prefix = rospy.search_param('tf_prefix')
        prefix_val = ''
        if tf_prefix is not None:  # prefix defined?
            prefix_val = rospy.get_param(tf_prefix)
            if prefix_val[0] != '/':  # prefix not absolute?
                prefix_val = '/' + prefix_val
        args['frame_id'] = prefix_val + '/' + args['frame_id']
    return args


if __name__ == "__main__":
    main()
