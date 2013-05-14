#!/usr/bin/env python
#
# Basic PTZ node, based on documentation here:
#   http://www.axis.com/files/manuals/vapix_ptz_45621_en_1112.pdf
#
import threading
import httplib, urllib
import rospy 
from axis_camera.msg import Axis
from std_msgs.msg import Bool
import math

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
        conn = httplib.HTTPConnection(self.axis.hostname)
        queryParams = { 'query':'position' }
        try:
            conn.request("GET", "/axis-cgi/com/ptz.cgi?%s" % 
                                                urllib.urlencode(queryParams))
            response = conn.getresponse()
            # Response code 200 means there are data to be read:
            if response.status == 200: 
                body = response.read()
                self.cameraPosition = dict([s.split('=',2) for s in 
                                                            body.splitlines()])
            else:
                self.cameraPosition = None
        except:
            rospy.logwarn('Encountered a problem getting a repsonse from %s.  '
                            'Possibly a problem with the network connection.' %
                            self.axis.hostname)
            self.cameraPosition = None
   
    def publishCameraState(self):
        '''Publish camera state to a ROS message'''
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
            self.msg.autofocus = (self.cameraPosition['autofocus'] == 'on')
            self.axis.pub.publish(self.msg)
            
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
    def __init__(self, hostname, username, password, flip, speed_control):
        self.hostname = hostname
        self.username = username
        self.password = password
        self.flip = flip
        # speed_control is true for speed control and false for
        # position control:
        self.speedControl = speed_control
        self.mirror = False

        self.st = None
        self.pub = rospy.Publisher("state", Axis, self)
        self.sub = rospy.Subscriber("cmd", Axis, self.cmd, queue_size=1)
        self.sub_mirror = rospy.Subscriber("mirror", Bool, self.mirrorCallback,
                                                                queue_size=1)

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
        conn = httplib.HTTPConnection(self.hostname)
        self.createCmdString()
        try:
            conn.request('GET', self.cmdString)
        except:
            rospy.logwarn('Failed to connect to camera to send command message')
            conn.close()

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
            self.cmdString += 'autoiris=on'
        else: # position control:
            self.cmdString += 'pan=%d&tilt=%d&' % (self.msg.pan, self.msg.tilt)\
                        + 'zoom=%d&' % (int(self.msg.zoom)) \
                        + 'brightness=%d&' % (int(self.msg.brightness))
            if self.msg.autofocus:
                self.cmdString += 'autofocus=on&'
            else:
                self.cmdString += 'autofocus=off&focus=%d&' % \
                                                        (int(self.msg.focus))
            self.cmdString += 'autoiris=on'

    def mirrorCallback(self, msg):
        '''Command the camera with speed control or position control commands'''
        self.mirror = msg.data

def main():
    rospy.init_node("axis_twist")

    arg_defaults = {
        'hostname': '192.168.0.90',
        'username': '',
        'password': '',
        'flip': True,
        'speed_control': False
        }
    args = {}
    for name in arg_defaults:
        args[name] = rospy.get_param(rospy.search_param(name), 
                                                            arg_defaults[name])

    AxisPTZ(**args)
    rospy.spin()

if __name__ == "__main__":
    main()
