#!/usr/bin/env python3
"""Pan/Tilt/Zoom ("PTZ") control classes for Axis cameras

This module contains the necessary web API calls to control the position & zoom of Axis cameras.
"""

import math
import os
import requests, requests.auth
import rospy
import subprocess
import threading
import urllib.parse

from axis_camera.cfg import PTZConfig
from axis_msgs.msg import Ptz
from std_msgs.msg import Bool

from math import degrees as rad2deg
from math import radians as deg2rad

## The Axis cameras have a maximum pan/tilt speed of 2.61 rad/s (150 deg/s)
MAX_ANGULAR_VELOCITY = 2.61

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

        while True:
            self.queryCameraPosition()
            self.publishCameraState()
            r.sleep()

    def waitForHost(self):
        '''Wait until the host is actually online before we try to contact it.
        This reduces http related errors'''

        # ping syntax is different on Windows than Linux, so set the command accordingly
        if os.name == 'nt':
            cmd = f"ping -W 5 -n 1 {self.axis.hostname}".split()
        else:
            cmd = f"ping -W 5 -c 1 {self.axis.hostname}".split()

        rospy.loginfo(f"Waiting until {self.axis.hostname} is online...")
        host_alive = subprocess.call(cmd) == 0
        rate = rospy.Rate(1)
        while not host_alive:
            rate.sleep()
            host_alive = subprocess.call(cmd) == 0

        rospy.loginfo(f"{self.axis.hostname} is now online")

    def publishCameraState(self):
        '''Publish camera state to a ROS message'''
        try:
            if self.cameraPosition is not None:
                msg = Ptz()
                msg.pan = deg2rad(-float(self.cameraPosition["pan"]))
                msg.tilt = deg2rad(float(self.cameraPosition["tilt"]))
                msg.zoom = float(self.cameraPosition["zoom"])

                if self.axis.flip:
                    self.adjustForFlippedOrientation(msg)
                if self.axis.mirror:
                    msg.pan = math.pi - msg.pan

                self.axis.pub.publish(msg)
                self.cameraPosition = None  # This prevents us re-publishing the same state on-error
        except KeyError as e:
            rospy.logwarn("Camera not ready for polling its telemetry: " + repr(e))

    def adjustForFlippedOrientation(self, msg):
        '''Correct pan and tilt parameters if camera is mounted backwards and
        facing down'''
        msg.pan = math.pi - msg.pan
        if msg.pan > math.pi:
            msg.pan -= 2*math.pi
        elif msg.pan < -math.pi:
            msg.pan += 2*math.pi
        msg.tilt = -msg.tilt

class AxisPTZ:
    '''This class creates a node to manage the PTZ functions of an Axis PTZ camera

    @param args  A dictionaru containing the rosparam values we need
    '''
    def __init__(self, axis_camera, args):
        self.axis_camera = axis_camera

        self.hostname = args['hostname']
        self.username = args['username']
        self.password = args['password']
        self.use_encrypted_password = args['use_encrypted_password']
        self.flip = False
        self.mirror = False

        self.st = None
        self.pub = rospy.Publisher("state/position", Ptz, self, queue_size=1)
        self.sub = rospy.Subscriber("cmd/position", Ptz, self.cmd_position, queue_size=1)
        self.sub = rospy.Subscriber("cmd/velocity", Ptz, self.cmd_velocity, queue_size=1)
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

    def cmd_position(self, msg):
        '''Command the camera with position control commands'''

        # flip & mirror as needed
        if self.flip:
            msg.tilt = - msg.tilt
            msg.pan = math.pi - msg.pan
        if self.mirror:
            msg.pan = -msg.pan

        self.sanitisePTZCommands(msg, False)
        self.applySetpoints(msg, False)

    def cmd_velocity(self, msg):
        '''Command the camera with speed control commands'''

        # flip & mirror as needed
        if self.flip:
            msg.tilt = - msg.tilt
            msg.pan = -msg.pan
        if self.mirror:
            msg.pan = -msg.pan

        self.sanitisePTZCommands(msg, True)
        self.applySetpoints(msg, True)

    def sanitisePTZCommands(self, msg, speedControl=False):
        '''Applies limits to message and corrects for flipped camera if
        necessary'''
        self.sanitisePan(msg, speedControl)
        self.sanitiseTilt(msg, speedControl)
        self.sanitiseZoom(msg, speedControl)

    def sanitisePan(self, msg, speedControl=False):
        '''Clamp the message's pan value to be in the valid range for the control mode

        Position: [-pi, pi]
        Velocity: [-2.61, 2.61]

        Certain models of camera may not be able to achieve full 360-degree rotation, e.g. many dome
        cameras are restricted to -170 to 170 degrees, but e.g. the the Q62 allows continuous panning through zero
        '''
        if speedControl:
            if msg.pan < -MAX_ANGULAR_VELOCITY:
                msg.pan = -MAX_ANGULAR_VELOCITY
            elif msg.pan > MAX_ANGULAR_VELOCITY:
                msg.pan = MAX_ANGULAR_VELOCITY
        else:
            if msg.pan < -math.pi:
                msg.pan = -math.pi
            elif msg.pan > math.pi:
                msg.pan = math.pi

    def sanitiseTilt(self, msg, speedControl=False):
        '''Clamp the message's tilt value to be in the valid range for the control mode

        Position: [-pi/2, pi/2]
        Velocity: [-2.61, 2.61]

        Certain models of camera may not have the full -90 to +90 tilt range, but some do
        '''
        if speedControl:
            if msg.tilt < -MAX_ANGULAR_VELOCITY:
                msg.tilt = -MAX_ANGULAR_VELOCITY
            elif msg.tilt > MAX_ANGULAR_VELOCITY:
                msg.tilt = MAX_ANGULAR_VELOCITY
        else:
            if msg.tilt < -math.pi/2:
                msg.tilt = -math.pi/2
            elif msg.tilt > math.pi/2:
                msg.tilt = math.pi/2

    def sanitiseZoom(self, msg, speedControl=False):
        '''Clamp the message's zoom value to be in the valid range for the control mode

        Position: [0, 9999]
        Velocity: [-100, 100]
        '''
        if speedControl:
            if abs(msg.zoom)>100:
                msg.zoom = math.copysign(100.0, msg.zoom)
        else:
            if msg.zoom>9999.0:
                msg.zoom = 9999.0
            elif msg.zoom<1.0:
                msg.zoom = 1.0

    def applySetpoints(self, msg, speedControl=False):
        '''Apply set-points to camera via HTTP'''

        self.createCmdString(msg, speedControl)
        try:
            url = f"http://{self.hostname}/{self.cmdString}"
            resp = requests.get(url, auth=self.http_auth, timeout=self.http_timeout, headers=self.http_headers)

            if resp.status_code != requests.status_codes.codes.ok:
                pass
            else:
                raise Exception(f"HTTP error {resp.status_code}")

        except Exception as e:
            rospy.logwarn(f'Failed to connect to camera to send command message: {e}')

    def createCmdString(self, msg, speedControl=False):
        '''Creates http cgi string to command PTZ camera'''
        self.cmdString = '/axis-cgi/com/ptz.cgi?'
        if speedControl:
            # externally we treat positive pan as anticlockwise, but the Axis API treats it as clockwise
            # we also need to rescale to [-100, 100] as a percentage of max speed
            pan_speed_percent = -msg.pan / MAX_ANGULAR_VELOCITY * 100
            tilt_speed_percent = msg.tilt / MAX_ANGULAR_VELOCITY * 100
            self.cmdString += f"continuouspantiltmove={int(pan_speed_percent)},{int(tilt_speed_percent)}&continuouszoommove={int(msg.zoom)}"

        else:
            # externally we treat positive angles as anticlockwise, but the Axis API treats them as clockwise
            # we also need to convert to degrees, since that's what the REST API uses
            pan_degrees = -rad2deg(msg.pan)
            tilt_degrees = rad2deg(msg.tilt)
            self.cmdString += f"pan={int(pan_degrees)}&tilt={int(tilt_degrees)}&zoom={int(msg.zoom)}"

    def mirrorCallback(self, msg):
        '''Command the camera with speed control or position control commands'''
        self.mirror = msg.data
