#!/usr/bin/env python3
#
# Basic PTZ node, based on documentation here:
#   http://www.axis.com/files/manuals/vapix_ptz_45621_en_1112.pdf
#
import threading
import requests, requests.auth
import urllib.parse
import rospy
from axis_camera.msg import Axis
from std_msgs.msg import Bool
import math
from dynamic_reconfigure.server import Server
from axis_camera.cfg import PTZConfig

class StateThread(threading.Thread):
    '''This class handles the publication of the positional state of the camera
    to a ROS message'''

    def __init__(self, axis):
        threading.Thread.__init__(self)
        self.axis = axis
        # Permit program to exit even if threads are still running by flagging
        # thread as a daemon:
        self.daemon = True

    def run(self):
        r = rospy.Rate(1)
        self.msg = Axis()

        while True:
            self.queryCameraPosition()
            self.publishCameraState()
            r.sleep()

    def queryCameraPosition(self):
        '''Using Axis VAPIX protocol, described in the comments at the top of
        this file, is used to query the state of the camera'''

        queryParams = { 'query':'position' }

        try:
            url = f"http://{self.axis.hostname}/axis-cgi/com/ptz.cgi?{urllib.parse.urlencode(queryParams)}"
            resp = requests.get(url, auth=self.axis.http_auth, timeout=self.axis.http_timeout, headers=self.axis.http_headers)

            if resp.status_code == requests.status_codes.codes.ok:
                # returns a string of the form
                #   pan=-0.01
                #   tilt=-45.03
                #   zoom=1
                #   iris=5748
                #   focus=4642
                #   brightness=4999
                #   autofocus=on
                #   autoiris=on
                new_camera_position = {}
                body = resp.text.split()
                for row in body:
                    if '=' in row:
                        (key, value) = row.split('=')
                        new_camera_position[key.strip()] = value.strip()

                self.cameraPosition = new_camera_position
            else:
                raise Exception(f"HTTP Error querying the camera position: {resp.status_code}")

        except Exception as e:
            exception_error_str = "Exception: '" + str(e) + "' when querying the url: http://" + \
                                  self.axis.hostname + "/axis-cgi/com/ptz.cgi?%s" % urllib.parse.urlencode(queryParams)
            rospy.logwarn(exception_error_str)

            self.cameraPosition = None

    def publishCameraState(self):
        '''Publish camera state to a ROS message'''
        try:
            if self.cameraPosition is not None:
                self.msg.pan = float(self.cameraPosition['pan'])
                if self.axis.flip:
                    self.adjustForFlippedOrientation()
                self.msg.tilt = float(self.cameraPosition['tilt'])
                self.msg.zoom = float(self.cameraPosition['zoom'])
                self.msg.brightness = float(self.cameraPosition['brightness'])
                self.msg.iris = 0.0
                if 'iris' in self.cameraPosition:
                    self.msg.iris = float(self.cameraPosition['iris'])
                self.msg.focus = 0.0
                if 'focus' in self.cameraPosition:
                    self.msg.focus = float(self.cameraPosition['focus'])
                if 'autofocus' in self.cameraPosition:
                    self.msg.autofocus = (self.cameraPosition['autofocus'] == 'on')
                if 'autoiris' in self.cameraPosition:
                    self.msg.autoiris = (self.cameraPosition['autoiris'] == 'on')
                self.axis.pub.publish(self.msg)
        except KeyError as e:
            rospy.logwarn("Camera not ready for polling its telemetry: " + repr(e.message))

    def adjustForFlippedOrientation(self):
        '''Correct pan and tilt parameters if camera is mounted backwards and
        facing down'''
        self.msg.pan = 180 - self.msg.pan
        if self.msg.pan > 180:
            self.msg.pan -= 360
        elif self.msg.pan < -180:
            self.msg.pan += 360
        self.msg.tilt = -self.msg.tilt

class AxisPTZ:
    '''This class creates a node to manage the PTZ functions of an Axis PTZ
    camera'''
    def __init__(self, hostname, username, password, use_encrypted_password, flip, speed_control):
        self.hostname = hostname
        self.username = username
        self.password = password
        self.use_encrypted_password = use_encrypted_password
        self.flip = flip
        # speed_control is true for speed control and false for
        # position control:
        self.speedControl = speed_control
        self.mirror = False

        self.st = None
        self.pub = rospy.Publisher("state", Axis, self, queue_size=1)
        self.sub = rospy.Subscriber("cmd", Axis, self.cmd, queue_size=1)
        self.sub_mirror = rospy.Subscriber("mirror", Bool, self.mirrorCallback,
                                                                queue_size=1)

        if self.use_encrypted_password:
            self.http_auth = requests.auth.HTTPDigestAuth(self.username, self.password)
        else:
            self.http_auth = requests.auth.HTTPBasicAuth(self.username, self.password)
        self.http_headers = {
            'User-Agent': 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/111.0.0.0 Safari/537.36',
            'From': f'http://{self.hostname}'
        }
        self.http_timeout = (3, 5)

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        '''Lazy-start the state publisher.'''
        if self.st is None:
            self.st = StateThread(self)
            self.st.start()

    def cmd(self, msg):
        '''Command the camera with speed control or position control commands'''
        self.msg = msg
        if self.flip:
            self.adjustForFlippedOrientation()
        if self.mirror:
            self.msg.pan = -self.msg.pan
        self.sanitisePTZCommands()
        self.applySetpoints()

    def adjustForFlippedOrientation(self):
        '''If camera is mounted backwards and upside down (ie. self.flip==True
        then apply appropriate transforms to pan and tilt'''
        self.msg.tilt = -self.msg.tilt
        if self.speedControl:
            self.msg.pan = -self.msg.pan
        else:
            self.msg.pan = 180.0 - self.msg.pan

    def sanitisePTZCommands(self):
        '''Applies limits to message and corrects for flipped camera if
        necessary'''
        self.sanitisePan()
        self.sanitiseTilt()
        self.sanitiseZoom()
        self.sanitiseFocus()
        self.sanitiseBrightness()
        self.sanitiseIris()

    def sanitisePan(self):
        '''Pan speed (in percent) must be: -100<pan<100'
        Pan must be: -180<pan<180 even though the Axis cameras can only achieve
        +/-170 degrees rotation.'''
        if self.speedControl:
            if abs(self.msg.pan)>100.0:
                self.msg.pan = math.copysign(100.0, self.msg.pan)
        else: # position control so need to ensure -180<pan<180:
            self.msg.pan = ((self.msg.pan + 180.0) % 360.0) - 180.0

    def sanitiseTilt(self):
        '''Similar to self.sanitisePan() but for tilt'''
        if self.speedControl:
            if abs(self.msg.tilt)>100.0:
                self.msg.tilt = math.copysign(100.0, self.msg.tilt)
        else: # position control so ensure tilt: -180<tilt<180:
            self.msg.tilt = ((self.msg.tilt + 180.0) % 360) - 180.0

    def sanitiseZoom(self):
        '''Zoom must be: 1<zoom<9999.  continuouszoommove must be:
        -100<zoom<100'''
        if self.speedControl:
            if abs(self.msg.zoom)>100:
                self.msg.zoom = math.copysign(100.0, self.msg.zoom)
        else: # position control:
            if self.msg.zoom>9999.0:
                self.msg.zoom = 9999.0
            elif self.msg.zoom<1.0:
                self.msg.zoom = 1.0

    def sanitiseFocus(self):
        '''Focus must be: 1<focus<9999.  continuousfocusmove: -100<rfocus<100'''
        if self.speedControl:
            if abs(self.msg.focus)>100.0:
                self.msg.focus = math.copysign(100.0, self.msg.focus)
        else: # position control:
            if self.msg.focus>9999.0:
                self.msg.focus = 9999.0
            elif self.msg.focus < 1.0:
                self.msg.focus = 1.0

    def sanitiseBrightness(self):
        '''Brightness must be: 1<brightness<9999.  continuousbrightnessmove must
        be: -100<rbrightness<100.  Note that it appears that the brightness
        cannot be adjusted on the Axis 214PTZ'''
        if self.speedControl:
            if abs(self.msg.brightness) > 100.0:
                self.msg.brightness = math.copysign(100.0, self.msg.brightness)
        else: # position control:
            if self.msg.brightness>9999.0:
                self.msg.brightness = 9999.0
            elif self.msg.brightness<1.0:
                self.msg.brightness = 1.0

    def sanitiseIris(self):
        '''Iris value is read only because autoiris has been set to "on"'''
        if self.msg.iris>0.000001:
            rospy.logwarn("Iris value is read-only.")

    def applySetpoints(self):
        '''Apply set-points to camera via HTTP'''

        self.createCmdString()
        try:
            url = f"http://{self.hostname}/{self.cmdString}"
            resp = requests.get(url, auth=self.http_auth, timeout=self.http_timeout, headers=self.http_headers)

            if resp.status_code != requests.status_codes.codes.ok:
                pass
            else:
                raise Exception(f"HTTP error {resp.status_code}")

        except Exception as e:
            rospy.logwarn(f'Failed to connect to camera to send command message: {e}')

    def createCmdString(self):
        '''creates http cgi string to command PTZ camera'''
        self.cmdString = '/axis-cgi/com/ptz.cgi?'
        if self.speedControl:
            self.cmdString += 'continuouspantiltmove=%d,%d&' % \
                                    (int(self.msg.pan), int(self.msg.tilt)) \
                    + 'continuouszoommove=%d&' % (int(self.msg.zoom)) \
                    + 'continuousbrightnessmove=%d&' % \
                                                    (int(self.msg.brightness))
            # Note that brightness adjustment has no effect for Axis 214PTZ.
            if self.msg.autofocus:
                self.cmdString += 'autofocus=on&'
            else:
                self.cmdString += 'autofocus=off&continuousfocusmove=%d&' % \
                                                        (int(self.msg.focus))
            if self.msg.autoiris:
                self.cmdString += 'autoiris=on'
            else:
                self.cmdString += 'autoiris=off'
        else: # position control:
            self.cmdString += 'pan=%d&tilt=%d&' % (self.msg.pan, self.msg.tilt)\
                        + 'zoom=%d&' % (int(self.msg.zoom)) \
                        + 'brightness=%d&' % (int(self.msg.brightness))
            if self.msg.autofocus:
                self.cmdString += 'autofocus=on&'
            else:
                self.cmdString += 'autofocus=off&focus=%d&' % \
                                                        (int(self.msg.focus))
            if self.msg.autoiris:
                self.cmdString += 'autoiris=on'
            else:
                self.cmdString += 'autoiris=off'

    def mirrorCallback(self, msg):
        '''Command the camera with speed control or position control commands'''
        self.mirror = msg.data

    def callback(self, config, level):
        #self.speedControl = config.speed_control

        # create temporary message and fill with data from dynamic reconfigure
        temp_msg = Axis()
        temp_msg.pan = config.pan
        temp_msg.tilt = config.tilt
        temp_msg.zoom = config.zoom
        temp_msg.focus = config.focus
        temp_msg.brightness = config.brightness
        temp_msg.autofocus = config.autofocus

        # check sanity and apply values
        self.cmd(temp_msg)

        # read sanitized values and update GUI
        config.pan = self.msg.pan
        config.tilt = self.msg.tilt
        config.zoom = self.msg.zoom
        config.focus = self.msg.focus
        config.brightness = self.msg.brightness
        config.autofocus = self.msg.autofocus

        # update GUI with sanitized values
        return config

def main():
    rospy.init_node("axis_twist")

    arg_defaults = {
        'hostname': '192.168.0.90',
        'username': 'root',
        'password': '',
        'use_encrypted_password': False,
        'flip': False,  # things get weird if flip=true
        'speed_control': False,
        }
    args = {}

    # go through all arguments
    for name in arg_defaults:
        full_param_name = rospy.search_param(name)
        # make sure argument was found (https://github.com/ros/ros_comm/issues/253)
        if full_param_name == None:
            args[name] = arg_defaults[name]
        else:
            args[name] = rospy.get_param(full_param_name, arg_defaults[name])

    # create new PTZ object and start dynamic_reconfigure server
    my_ptz = AxisPTZ(**args)
    srv = Server(PTZConfig, my_ptz.callback)
    rospy.spin()

if __name__ == "__main__":
    main()
