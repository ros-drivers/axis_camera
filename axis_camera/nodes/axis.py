#!/usr/bin/env python3
#
# Axis camera image driver. Based on:
# https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/sandbox/axis_camera
# /axis.py
#

import camera_info_manager
import datetime
import os
import requests, requests.auth
import rospy
import subprocess
import threading
import time
import urllib.request, urllib.error, urllib.parse

from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

class StreamThread(threading.Thread):
    def __init__(self, axis):
        threading.Thread.__init__(self)
        self.axis = axis
        self.daemon = True
        self.timeoutSeconds = 2.5

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

    def run(self):
        self.waitForHost()

        while(True):
            self.stream()

    def stream(self):
        while(True):
            self.formURL()
            self.authenticate()
            if self.openURL():
                self.publishFramesContinuously()
            rospy.sleep(2) # if stream stays intact we shouldn't get to this

    def formURL(self):
        self.url = 'http://%s/mjpg/video.mjpg' % self.axis.hostname
        self.url += "?fps=%d&resolution=%dx%d" % (self.axis.fps, self.axis.width,
                                                            self.axis.height)

        # support for Axis F34 multicamera switch
        if (self.axis.camera != 0):
            self.url += "&camera=%s" % str(self.axis.camera)

        rospy.logdebug('opening ' + str(self.axis))

    def authenticate(self):
        '''only try to authenticate if user/pass configured.  I have not
        used this method (yet).'''
        if self.axis.password != '' and self.axis.username != '':
            # create a password manager
            password_mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()

            # Add the username and password, use default realm.
            top_level_url = "http://" + self.axis.hostname
            password_mgr.add_password(None, top_level_url, self.axis.username,
                                                            self.axis.password)
            if self.axis.use_encrypted_password:
                handler = urllib.request.HTTPDigestAuthHandler(password_mgr)
            else:
                handler = urllib.request.HTTPBasicAuthHandler(password_mgr)

            # create "opener" (OpenerDirector instance)
            opener = urllib.request.build_opener(handler)

            # ...and install it globally so it can be used with urlopen.
            urllib.request.install_opener(opener)

    def openURL(self):
        '''Open connection to Axis camera using http'''
        try:
            self.fp = urllib.request.urlopen(self.url, timeout=self.timeoutSeconds)
            return(True)
        except urllib.error.URLError as e:
            rospy.logwarn('Error opening URL %s' % (self.url) +
                            'Possible timeout.  Looping until camera appears')
            return(False)

    def publishFramesContinuously(self):
        '''Continuous loop to publish images'''
        while(True):
            try:
                self.findBoundary()
                self.getImage()
                self.publishMsg()
                self.publishCameraInfoMsg()

            except:
                rospy.logwarn('Timed out while trying to get message.')
                break

    def findBoundary(self):
        '''The string "--myboundary" is used to denote the start of an image in
        Axis cameras'''
        while(True):
            boundary = self.fp.readline()
            if boundary == b'--myboundary\r\n':
                break

    def getImage(self):
        '''Get the image header and image itself'''
        self.getHeader()
        self.getImageData()

    def getHeader(self):
        self.header = {}
        while(True):
            line = self.fp.readline()
            if line == b'\r\n':
                break
            line = line.strip()
            parts = line.split(b": ", 1)
            try:
                self.header[parts[0]] = parts[1]
            except:
                rospy.logwarn('Problem encountered with image header.  Setting '
                                                    'content_length to zero')
                self.header[b'Content-Length'] = 0 # set content_length to zero if
                                            # there is a problem reading header
        self.content_length = int(self.header[b'Content-Length'])

    def getImageData(self):
        '''Get the binary image data itself (ie. without header)'''
        if self.content_length>0:
            self.img = self.fp.read(self.content_length)
            self.fp.readline() # Read terminating \r\n and do nothing with it

    def publishMsg(self):
        '''Publish jpeg image as a ROS message'''
        self.msg = CompressedImage()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = self.axis.frame_id
        self.msg.format = "jpeg"
        self.msg.data = self.img
        self.axis.pub.publish(self.msg)

    def publishCameraInfoMsg(self):
        '''Publish camera info manager message'''
        cimsg = self.axis.cinfo.getCameraInfo()
        cimsg.header.stamp = self.msg.header.stamp
        cimsg.header.frame_id = self.axis.frame_id
        cimsg.width = self.axis.width
        cimsg.height = self.axis.height
        self.axis.caminfo_pub.publish(cimsg)

class Axis:
    def __init__(self, hostname, username, password, width, height, fps, frame_id,
                 camera_info_url, use_encrypted_password, camera, ir, defog, wiper):
        self.hostname = hostname
        self.username = username
        self.password = password
        self.width = width
        self.height = height
        self.fps = fps
        self.frame_id = frame_id
        self.camera_info_url = camera_info_url
        self.use_encrypted_password = use_encrypted_password
        self.camera = camera

        self.http_headers = {
            'User-Agent': 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/111.0.0.0 Safari/537.36',
            'From': f'http://{self.hostname}'
        }
        if self.use_encrypted_password:
            self.http_auth = requests.auth.HTTPDigestAuth(self.username, self.password)
        else:
            self.http_auth = requests.auth.HTTPBasicAuth(self.username, self.password)
        self.http_timeout = (3, 5)

        # generate a valid camera name based on the hostname
        self.cname = camera_info_manager.genCameraName(self.hostname)
        self.cinfo = camera_info_manager.CameraInfoManager(cname = self.cname,
                                                   url = self.camera_info_url)
        self.cinfo.loadCameraInfo()         # required before getCameraInfo()
        self.st = None
        self.pub = rospy.Publisher("image_raw/compressed", CompressedImage, self, queue_size=1)
        self.caminfo_pub = rospy.Publisher("camera_info", CameraInfo, self, queue_size=1)

        # The Axis Q62 series supports a night-vision mode with an active IR illuminator
        # If this option is enabled, add the necessary services and topics
        if ir:
            self.ir_on = False
            self.ir_on_off_srv = rospy.Service('set_ir_on', SetBool, self.handle_toggle_ir)
            self.ir_on_pub = rospy.Publisher('ir_on', Bool, queue_size=1)
            self.ir_on_pub_thread = threading.Thread(target=self.ir_on_pub_thread_fn)
            self.ir_on_pub_thread.start()

            self.handle_toggle_ir(SetBoolRequest(False))

        # The Axis Q62 series is equipped with a wiper on the camera lens
        # If this option is enabled, add the necessary services and topics
        if wiper:
            self.wiper_on_time = datetime.datetime.utcnow()
            self.wiper_on = False
            self.wiper_on_off_srv = rospy.Service('set_wiper_on', SetBool, self.handle_toggle_wiper)
            self.wiper_on_pub = rospy.Publisher('wiper_on', Bool, queue_size=1)
            self.wiper_on_pub_thread = threading.Thread(target=self.wiper_on_pub_thread_fn)
            self.wiper_on_pub_thread.start()

            self.handle_toggle_wiper(SetBoolRequest(False))

        # The Axis Q62 series is equipped with a defogger
        # If this option is enabled, add the necessary services and topics
        if defog:
            self.defog_on = False
            self.defog_on_off_srv = rospy.Service('set_defog_on', SetBool, self.handle_toggle_defog)
            self.defog_on_pub = rospy.Publisher('defog_on', Bool, queue_size=1)
            self.defog_on_pub_thread = threading.Thread(target=self.defog_on_pub_thread_fn)
            self.defog_on_pub_thread.start()

            self.handle_toggle_defog(SetBoolRequest(False))

    def __str__(self):
        """Return string representation."""
        return(self.hostname + ',' + self.username + ',' + self.password +
                       '(' + str(self.width) + 'x' + str(self.height) + ')')

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        '''Lazy-start the image-publisher.'''
        if self.st is None:
            self.st = StreamThread(self)
            self.st.start()

    def handle_toggle_ir(self, req):
        """Turn the IR mode on/off (if supported)"""
        resp = SetBoolResponse()
        resp.success = True
        on_off = {
            True: "on",
            False: "off"
        }
        try:
            # Set the IR led on/off as needed
            if req.data:
                post_data = '{"apiVersion": "1.0", "method": "enableLight", "params": {"lightID": "led0"}}'
            else:
                post_data = '{"apiVersion": "1.0", "method": "disableLight", "params": {"lightID": "led0"}}'
            http_resp = requests.post(f"http://{self.hostname}/axis-cgi/lightcontrol.cgi",  post_data,
                auth=self.http_auth,
                headers=self.http_headers,
                timeout=self.http_timeout)

            if http_resp.status_code != requests.status_codes.codes.ok:
                raise Exception(f"HTTP Error setting IR illuminator: {http_resp.status_code}")

            # Enable/disable the IR filter
            # OLD:
            #   http://192.168.131.10/axis-cgi/param.cgi?action=update&PTZ.Various.V1.IrCutFilter=off&timestamp=$(date +'%s')
            # NEW:
            #   http://192.168.131.10/axis-cgi/param.cgi?action=update&ImageSource.I0.DayNight.IrCutFilter=no&timestamp=$(date +'%s')
            if req.data:
                get_url = f"http://{self.hostname}/axis-cgi/param.cgi?action=update&PTZ.Various.V1.IrCutFilter=off&timestamp={int(time.time())}"
            else:
                get_url = f"http://{self.hostname}/axis-cgi/param.cgi?action=update&PTZ.Various.V1.IrCutFilter=on&timestamp={int(time.time())}"
            http_resp = requests.get(get_url,
                auth=self.http_auth,
                headers=self.http_headers,
                timeout=self.http_timeout)

            if http_resp.status_code != requests.status_codes.codes.ok:
                raise Exception(f"HTTP Error setting IR filter: {http_resp.status_code}")

            resp.message = f"IR mode is {on_off[req.data]}"
            self.ir_on = req.data
        except Exception as err:
            rospy.logwarn(f"Failed to set IR mode: {err}")
            ok = False
            resp.message = str(err)

        return resp

    def ir_on_pub_thread_fn(self):
        """Publish whether the IR mode is on or off at 1Hz"""
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.ir_on_pub.publish(Bool(self.ir_on))
            rate.sleep()

    def handle_toggle_wiper(self, req):
        """Turn the wiper on/off (if supported)"""
        on_off = {
            True: "on",
            False: "off"
        }

        resp = SetBoolResponse()
        resp.success = True
        try:
            if req.data:
                post_data = '{"apiVersion": "1.0", "context": "lvc_context", "method": "start", "params": {"id": 0, "duration": 10}}'
                self.wiper_on_time = datetime.datetime.utcnow()
            else:
                post_data = '{"apiVersion": "1.0", "context": "lvc_context", "method": "stop", "params": {"id": 0}}'

            http_resp = requests.post(f"http://{self.hostname}/axis-cgi/clearviewcontrol.cgi", post_data,
                auth=self.http_auth,
                headers=self.http_headers,
                timeout=self.http_timeout)

            if http_resp.status_code != requests.status_codes.codes.ok:
                raise Exception(f"HTTP Error setting wiper: {http_resp.status_code}")

            resp.message = f"Wiper is {on_off[self.wiper_on]}"
            self.wiper_on = req.data
        except Exception as err:
            rospy.logwarn(f"Failed to set wiper mode: {err}")
            resp.success = False
            resp.message = str(err)
        return resp

    def wiper_on_pub_thread_fn(self):
        """Publish whether the wiper is running or not at 1Hz"""
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            # the wiper shuts off automatically after 10s
            if (datetime.datetime.utcnow() - self.wiper_on_time).total_seconds() > 10:
                self.wiper_on = False

            self.wiper_on_pub.publish(Bool(self.wiper_on))
            rate.sleep()

    def handle_toggle_defog(self, req):
        """Turn the defogger on/off (if supported)"""
        on_off = {
            True: "on",
            False: "off"
        }

        resp = SetBoolResponse()
        resp.success = True
        try:
            if req.data:
                get_url = f"http://{self.hostname}/axis-cgi/param.cgi?action=update&ImageSource.I0.Sensor.Defog=on&timestamp={int(time.time())}"
            else:
                get_url = f"http://{self.hostname}/axis-cgi/param.cgi?action=update&ImageSource.I0.Sensor.Defog=off&timestamp={int(time.time())}"

            http_resp = requests.get(get_url,
                auth=self.http_auth,
                headers=self.http_headers,
                timeout=self.http_timeout)

            if http_resp.status_code != requests.status_codes.codes.ok:
                raise Exception(f"HTTP Error setting defogger: {http_resp.status_code}")

            resp.message = f"Defogger is {on_off[self.defog_on]}"
            self.defog_on = req.data
        except Exception as err:
            rospy.logwarn(f"Failed to set defogger mode: {err}")
            resp.success = False
            resp.message = str(err)
        return resp

    def defog_on_pub_thread_fn(self):
        """Publish the state of the defogger at 1Hz"""
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.defog_on_pub.publish(Bool(self.defog_on))
            rate.sleep()

def main():
    rospy.init_node("axis_driver")

    arg_defaults = {
        'hostname': '192.168.0.90',       # default IP address
        'username': 'root',               # default login name
        'password': '',
        'width': 640,
        'height': 480,
        'fps': 0,                         # frames per second (0 = camera default)
        'frame_id': 'axis_camera',
        'camera_info_url': '',
        'use_encrypted_password' : True,
        'camera' : 0,
        'ir': False,
        'defog': False,
        'wiper': False }
    args = updateArgs(arg_defaults)
    Axis(**args)
    rospy.spin()

def updateArgs(arg_defaults):
    '''Look up parameters starting in the driver's private parameter space, but
    also searching outer namespaces.  Defining them in a higher namespace allows
    the axis_ptz.py script to share parameters with the driver.'''
    args = {}
    for name, val in arg_defaults.items():
        full_name = rospy.search_param(name)
        if full_name is None:
            args[name] = val
        else:
            args[name] = rospy.get_param(full_name, val)
    # resolve frame_id with tf_prefix (unless already absolute)
    if args['frame_id'][0] != '/':        # not absolute?
        tf_prefix = rospy.search_param('tf_prefix')
        prefix_val = ''
        if tf_prefix is not None:           # prefix defined?
            prefix_val = rospy.get_param(tf_prefix)
            if prefix_val[0] != '/':          # prefix not absolute?
                prefix_val = '/' + prefix_val
        args['frame_id'] = prefix_val + '/' + args['frame_id']
    return(args)

if __name__ == "__main__":
    main()
