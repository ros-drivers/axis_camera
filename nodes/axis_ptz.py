#!/usr/bin/env python
#
# Basic VAPIX PTZ node, based on documentation here:
#   http://www.axis.com/global/en/support/developer-support/vapix
#
import threading
import httplib, urllib
import urllib2
import rospy 
from axis_camera.msg import Axis, PTZ
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import math
from dynamic_reconfigure.server import Server
from axis_camera.cfg import PTZConfig

from axis_camera.vapix import VAPIX
from axis_camera.position_streaming import PositionStreamingThread
from axis_camera.camera_control import AxisCameraController

StateThread = PositionStreamingThread


class AxisPTZ:
    """This class creates a node to manage the PTZ functions of an Axis PTZ camera"""
    def __init__(self, hostname, username, password, flip, speed_control, frame_id="axis_camera",
                 use_encrypted_password=False, state_publishing_frequency=1, camera_id=1):

        self.hostname = hostname
        self.camera_id = camera_id
        self.frame_id = frame_id

        self.connection_timeout = 5
        self.state_publishing_frequency = state_publishing_frequency

        self.api = None
        # autodetect the VAPIX API and connect to it; try it forever
        while self.api is None and not rospy.is_shutdown():
            try:
                self.api = VAPIX.get_api_for_camera(hostname, username, password, camera_id, use_encrypted_password)
            except (IOError, ValueError):
                rospy.loginfo(
                    "Retrying connection to VAPIX on host %s, camera %d in 2 seconds." % (hostname, camera_id))
                rospy.sleep(2)
        if rospy.is_shutdown():
            return

        if not self.api.has_ptz():
            raise RuntimeError("Camera %d on host %s doesn't have a Pan-Tilt-Zoom unit." % (self.camera_id, self.hostname))

        # Create a controller of the camera
        self.camera_controller = AxisCameraController(self.api, flip_vertically=flip, flip_horizontally=flip)

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

        # Needs to be after the backwards compatibility setup
        # start the publisher thread
        self.publisher_thread = PositionStreamingThread(self, self.api)
        self.st = self.publisher_thread  # deprecated
        self.publisher_thread.start()

    # BACKWARDS COMPATIBILITY LAYER

    def cmd(self, message):
        """Command the camera with speed control or position control commands"""
        self.msg = message

        self.sanitisePTZCommands()

        self.camera_controller.set_ptz(message.pan, message.tilt, message.zoom)
        self.camera_controller.set_autofocus(message.autofocus)
        if not message.autofocus:
            self.camera_controller.set_focus(message.focus)
        self.camera_controller.set_autoiris(True)
        self.camera_controller.set_brightness(message.brightness)

    def adjustForFlippedOrientation(self):
        '''If camera is mounted backwards and upside down (ie. self.flip==True
        then apply appropriate transforms to pan and tilt'''
        self.msg.tilt = -self.msg.tilt
        if self.speedControl:
            self.msg.pan = -self.msg.pan
        else:
            self.msg.pan = 180.0 - self.msg.pan
    
    def sanitisePTZCommands(self):
        """Applies limits to message and corrects for flipped camera if
        necessary"""
        if not self.speedControl:
            self.msg.pan = self.api.ptz_limits['Pan'].absolute.crop_value(self.msg.pan)
            self.msg.tilt = self.api.ptz_limits['Tilt'].absolute.crop_value(self.msg.tilt)
            self.msg.zoom = self.api.ptz_limits['Zoom'].absolute.crop_value(self.msg.zoom)
            self.msg.focus = self.api.ptz_limits['Focus'].absolute.crop_value(self.msg.focus)
            self.msg.brightness = self.api.ptz_limits['Brightness'].absolute.crop_value(self.msg.brightness)
            self.msg.iris = self.api.ptz_limits['Iris'].absolute.crop_value(self.msg.iris)
        else:
            self.msg.pan = self.api.ptz_limits['Pan'].velocity.crop_value(self.msg.pan)
            self.msg.tilt = self.api.ptz_limits['Tilt'].velocity.crop_value(self.msg.tilt)
            self.msg.zoom = self.api.ptz_limits['Zoom'].velocity.crop_value(self.msg.zoom)
            self.msg.focus = self.api.ptz_limits['Focus'].velocity.crop_value(self.msg.focus)
            self.msg.brightness = self.api.ptz_limits['Brightness'].velocity.crop_value(self.msg.brightness)
            self.msg.iris = self.api.ptz_limits['Iris'].velocity.crop_value(self.msg.iris)

    def sanitisePan(self):
        if self.speedControl:
            self.msg.pan = self.api.ptz_limits['Pan'].velocity.crop_value(self.msg.pan)
        else:
            self.msg.pan = self.api.ptz_limits['Pan'].absolute.crop_value(self.msg.pan)

    def sanitiseTilt(self):
        if self.speedControl:
            self.msg.tilt = self.api.ptz_limits['Tilt'].velocity.crop_value(self.msg.tilt)
        else:
            self.msg.tilt = self.api.ptz_limits['Tilt'].absolute.crop_value(self.msg.tilt)

    def sanitiseZoom(self):
        if self.speedControl:
            self.msg.zoom = self.api.ptz_limits['Zoom'].velocity.crop_value(self.msg.zoom)
        else:
            self.msg.zoom = self.api.ptz_limits['Zoom'].absolute.crop_value(self.msg.zoom)
        
    def sanitiseFocus(self):
        if self.speedControl:
            self.msg.focus = self.api.ptz_limits['Focus'].velocity.crop_value(self.msg.focus)
        else:
            self.msg.focus = self.api.ptz_limits['Focus'].absolute.crop_value(self.msg.focus)
            
    def sanitiseBrightness(self):
        if self.speedControl:
            self.msg.brightness = self.api.ptz_limits['Brightness'].velocity.crop_value(self.msg.brightness)
        else:
            self.msg.brightness = self.api.ptz_limits['Brightness'].absolute.crop_value(self.msg.brightness)

    def sanitiseIris(self):
        if self.msg.iris > 0.000001:
            rospy.logwarn("Iris value is read-only.")

    def applySetpoints(self):
        """Apply the command to the camera using the HTTP API"""
        self.camera_controller.set_ptz(self.msg.pan, self.msg.tilt, self.msg.zoom)
        self.camera_controller.set_autofocus(self.msg.autofocus)
        if not self.msg.autofocus:
            self.camera_controller.set_focus(self.msg.focus)
        self.camera_controller.set_autoiris(True)
        self.camera_controller.set_brightness(self.msg.brightness)

    def createCmdString(self):
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
        '''Command the camera with speed control or position control commands'''
        self.mirror = msg.data
        self.camera_controller.mirror_horizontally = self.mirror
        
    def callback(self, config, level):
        #self.speedControl = config.speed_control
        
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
        config.focus = self.camera_controller.focus
        config.brightness = self.camera_controller.brightness
        config.autofocus = self.camera_controller.autofocus
        
        # update GUI with sanitized values
        return config


def main():
    rospy.init_node("axis_ptz_driver")

    arg_defaults = {
        'hostname': '192.168.0.90',
        'username': '',
        'password': '',
        'flip': False,
        'speed_control': False,
        'frame_id': 'axis_camera',
        'use_encrypted_password': False,
        'state_publishing_frequency': 1,
        'camera_id': 1,
        }
    args = read_args_with_defaults(arg_defaults)

    # Start the driver
    my_ptz = AxisPTZ(**args)

    srv = Server(PTZConfig, my_ptz.callback)  # deprecated

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
