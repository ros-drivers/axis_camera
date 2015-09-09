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

        self.flip = flip
        # speed_control is true for speed control and false for
        # position control:
        self.speedControl = speed_control
        self.mirror = False
        self.state_publishing_frequency = state_publishing_frequency

        self.connection_timeout = 5

        self.camera_controller = AxisCameraController(self.api)

        self.state_publisher = rospy.Publisher("camera/ptz", PTZ, queue_size=100)
        self.joint_states_publisher = rospy.Publisher("camera/joint_states", JointState, queue_size=100)
        self.command_subscriber = rospy.Subscriber("cmd", Axis, self.command_callback, queue_size=100)
        self.mirror_subscriber = rospy.Subscriber("mirror", Bool, self.mirrorCallback, queue_size=100)

        self.dynamic_reconfigure_server = Server(PTZConfig, self.reconfigure)

        # BACKWARDS COMPATIBILITY LAYER
        self.username = username  # deprecated
        self.password = password  # deprecated
        self.pub = rospy.Publisher("state", Axis, queue_size=100)

        # start the publisher thread
        self.publisher_thread = PositionStreamingThread(self, self.api)
        self.publisher_thread.start()

    def command_callback(self, message):
        """Command the camera with speed control or position control commands"""
        if self.flip:
            self.adjustForFlippedOrientation(message)
        if self.mirror:
            message.pan = -message.pan

        self.sanitisePTZCommands(message)
        self.apply_command_to_camera(message)

    def adjustForFlippedOrientation(self, message):
        """If camera is mounted backwards and upside down (ie. self.flip==True
        then apply appropriate transforms to pan and tilt"""
        message.tilt = -message.tilt
        if self.speedControl:
            message.pan = -message.pan
        else:
            message.pan = 180.0 - message.pan
    
    def sanitisePTZCommands(self, message):
        """Applies limits to message and corrects for flipped camera if
        necessary"""
        self.sanitisePan(message)
        self.sanitiseTilt(message)
        self.sanitiseZoom(message)
        self.sanitiseFocus(message)
        self.sanitiseBrightness(message)
        self.sanitiseIris(message)

    def sanitisePan(self, message):
        """Pan speed (in percent) must be: -100<pan<100'
        Pan must be: -180<pan<180 even though the Axis cameras can only achieve
        +/-170 degrees rotation."""
        if self.speedControl:
            if abs(message.pan) > 100.0:
                message.pan = math.copysign(100.0, message.pan)
        else: # position control so need to ensure -180<pan<180:
            message.pan = ((message.pan + 180.0) % 360.0) - 180.0

    def sanitiseTilt(self, message):
        '''Similar to self.sanitisePan() but for tilt'''
        if self.speedControl:
            if abs(message.tilt) > 100.0:
                message.tilt = math.copysign(100.0, message.tilt)
        else: # position control so ensure tilt: -180<tilt<180:
            message.tilt = ((message.tilt + 180.0) % 360) - 180.0

    def sanitiseZoom(self, message):
        '''Zoom must be: 1<zoom<9999.  continuouszoommove must be: 
        -100<zoom<100'''
        if self.speedControl:
            if abs(message.zoom) > 100:
                message.zoom = math.copysign(100, message.zoom)
        else: # position control:
            if message.zoom > 9999:
                message.zoom = 9999
            elif message.zoom < 1:
                message.zoom = 1
        message.zoom = int(message.zoom)
        
    def sanitiseFocus(self, message):
        '''Focus must be: 1<focus<9999.  continuousfocusmove: -100<rfocus<100'''
        if self.speedControl:
            if abs(message.focus) > 100.0:
                message.focus = math.copysign(100, message.focus)
        else: # position control:
            if message.focus > 9999:
                message.focus = 9999
            elif message.focus < 1:
                message.focus = 1
        message.focus = int(message.focus)
            
    def sanitiseBrightness(self, message):
        '''Brightness must be: 1<brightness<9999.  continuousbrightnessmove must
        be: -100<rbrightness<100.  Note that it appears that the brightness
        cannot be adjusted on the Axis 214PTZ'''
        if self.speedControl:
            if abs(message.brightness) > 100:
                message.brightness = math.copysign(100, message.brightness)
        else: # position control:
            if message.brightness > 9999:
                message.brightness = 9999
            elif message.brightness < 1:
                message.brightness = 1
        message.brightness = int(message.brightness)

    def sanitiseIris(self, message):
        '''Iris value is read only because autoiris has been set to "on"'''
        if message.iris > 0.000001:
            rospy.logwarn("Iris value is read-only.")

    def apply_command_to_camera(self, command):
        """Apply the command to the camera using the HTTP API"""
        url = self.construct_command_string_from_message(command)
        stream = self.open_url(url, valid_statuses=[200, 204])
        if stream is None:
            rospy.logwarn('Failed to connect to camera to send command message')

    def open_url(self, url, valid_statuses=None):
        """Open connection to Axis camera using http"""
        try:
            rospy.logdebug('Opening ' + url)
            stream = urllib2.urlopen(url, timeout=self.connection_timeout)
            if stream is not None and (valid_statuses is None or stream.getcode() in valid_statuses):
                return stream
            else:
                return None
        except urllib2.URLError, e:
            rospy.logwarn('Error opening URL %s' % url + 'Possible timeout.')
            return None

    def construct_command_string_from_message(self, message):
        """Created tje HTTP API string to command PTZ camera"""
        result = 'http://%s/axis-cgi/com/ptz.cgi?' % self.hostname
        if self.speedControl:
            result += 'continuouspantiltmove=%d,%d&' % (int(message.pan), int(message.tilt))
            result += 'continuouszoommove=%d&' % message.zoom
            result += 'continuousbrightnessmove=%d&' % message.brightness
            # Note that brightness adjustment has no effect for Axis 214PTZ.
            if message.autofocus:
                result += 'autofocus=on&'
            else:
                result += 'autofocus=off&continuousfocusmove=%d&' % message.focus
            result += 'autoiris=on'
        else: # position control:
            result += 'pan=%f&tilt=%f&' % (message.pan, message.tilt)
            result += 'zoom=%d&' % message.zoom
            result += 'brightness=%d&' % message.brightness
            if message.autofocus:
                result += 'autofocus=on&'
            else:
                result += 'autofocus=off&focus=%d&' % message.focus
            result += 'autoiris=on'

        return result

    def mirrorCallback(self, msg):
        '''Command the camera with speed control or position control commands'''
        self.mirror = msg.data
        
    def reconfigure(self, config, level):
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
        self.command_callback(command)
        
        # read sanitized values and update GUI
        config.pan = command.pan
        config.tilt = command.tilt
        config.zoom = command.zoom
        config.focus = command.focus
        config.brightness = command.brightness
        config.autofocus = command.autofocus
        
        # update GUI with sanitized values
        return config


def main():
    rospy.init_node("axis_ptz_driver")

    arg_defaults = {
        'hostname': '192.168.0.90',
        'username': '',
        'password': '',
        'flip': False,  # things get weird if flip=true
        'speed_control': False,
        'frame_id': 'axis_camera',
        'use_encrypted_password': False,
        'state_publishing_frequency': 1,
        'camera_id': 1,
        }
    args = read_args_with_defaults(arg_defaults)

    # Start the driver
    my_ptz = AxisPTZ(**args)

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
