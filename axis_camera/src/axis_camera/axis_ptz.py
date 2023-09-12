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
from axis_msgs.msg import Axis
from std_msgs.msg import Bool

from math import degrees as rad2deg

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
                new_camera_position = {
                    'pan': 0,
                    'tilt': 0,
                    'zoom': 1,
                    'iris': 0,
                    'focus': 0,
                    'brightness': 0,
                    'autofocus': 'off',
                    'autoiris': 'off'
                }
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
    '''This class creates a node to manage the PTZ functions of an Axis PTZ
    camera'''
    def __init__(self, hostname, username, password, use_encrypted_password, flip):
        self.hostname = hostname
        self.username = username
        self.password = password
        self.use_encrypted_password = use_encrypted_password
        self.flip = flip
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
        Velocity: [-100, 100]

        Certain models of camera may not be able to achieve full 360-degree rotation, e.g. many dome
        cameras are restricted to -170 to 170 degrees, but e.g. the the Q62 allows continuous panning through zero
        '''
        if speedControl:
            if abs(msg.pan)>100.0:
                msg.pan = math.copysign(100.0, msg.pan)
        else:
            if msg.pan < -math.pi:
                msg.pan = -math.pi
            elif msg.pan > math.pi:
                msg.pan = math.pi

    def sanitiseTilt(self, msg, speedControl=False):
        '''Clamp the message's tilt value to be in the valid range for the control mode

        Position: [-pi/2, pi/2]
        Velocity: [-100, 100]

        Certain models of camera may not have the full -90 to +90 tilt range, but some do
        '''
        if speedControl:
            if abs(msg.tilt)>100.0:
                msg.tilt = math.copysign(100.0, msg.tilt)
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
        if self.speedControl:
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
            self.cmdString += f"continuouspantiltmove={int(-msg.pan)},{int(msg.tilt)}&continuouszoommove={int(msg.zoom)}"

        else:
            # externally we treat positive angles as anticlockwise, but the Axis API treats them as clockwise
            pan_degrees = -rad2deg(msg.pan)
            tilt_degrees = rad2deg(msg.tilt)
            self.cmdString += f"pan={int(pan_degrees)}&tilt={int(tilt_degrees)}&zoom={int(msg.zoom)}"

    def mirrorCallback(self, msg):
        '''Command the camera with speed control or position control commands'''
        self.mirror = msg.data
