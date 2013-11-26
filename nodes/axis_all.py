#!/usr/bin/env python
#
# Basic PTZ node, based on documentation here:
#   http://www.axis.com/files/manuals/vapix_ptz_45621_en_1112.pdf
#

import os, sys, string, time, getopt, threading
import httplib, urllib

import roslib; roslib.load_manifest('axis_camera') 
import rospy 

from axis_camera.msg import Axis
from geometry_msgs.msg import Twist
import math


class StateThread(threading.Thread):
  def __init__(self, axis):
    threading.Thread.__init__(self)

    self.axis = axis
    self.daemon = True

  def run(self):
    r = rospy.Rate(10)
    msg = Axis()

    while True:
      if not self.axis.twist_timeout and ((rospy.Time.now() - self.axis.last_request).to_sec() > 1.0):
          self.axis.twist_timeout = True
          self.axis.cmd_twist(Twist(),reset_timeout=False)
      conn = httplib.HTTPConnection(self.axis.hostname)
      params = { 'query':'position' }
      conn.request("GET", "/axis-cgi/com/ptz.cgi?%s" % urllib.urlencode(params))
      response = conn.getresponse()
      if response.status == 200:
        body = response.read()
        msg.fields = msg.SELECT_PAN | msg.SELECT_TILT | msg.SELECT_ZOOM  \
            | msg.SELECT_AUTOFOCUS | msg.SELECT_IRIS
        params = dict([s.split('=',2) for s in body.splitlines()])
        msg.pan = float(params['pan'])
        # Flip pan orient if the camera is mounted backwards and facing down
        if self.axis.flip:
            msg.pan = 180 - msg.pan
            if msg.pan > 180:
                msg.pan -= 360
            if msg.pan < -180:
                msg.pan += 360
        msg.tilt = float(params['tilt'])
        msg.zoom = float(params['zoom'])
        msg.iris = float(params['iris'])
        msg.focus = 0.0
        if 'focus' in params:
            msg.fields = msg.fields | msg.FOCUS
            msg.focus = float(params['focus'])
        msg.autofocus = (params['autofocus'] == 'on')
        self.axis.pub.publish(msg)
      r.sleep()
      

class AxisPTZ:
  def __init__(self, hostname, username, password, flip, delay, max_command_rate):
    self.hostname = hostname
    self.username = username
    self.password = password
    self.max_command_rate = max_command_rate
    self.flip = flip

    self.st = None
    self.last_cmd = rospy.Time.now()
    self.twist_timeout = True
    self.pub = rospy.Publisher("state", Axis, self)
    self.sub = rospy.Subscriber("twist", Twist, self.cmd_twist)
    self.sub = rospy.Subscriber("cmd", Axis, self.cmd_abs)
    self.cmd_twist(Twist(),reset_timeout=False)

  def peer_subscribe(self, topic_name, topic_publish, peer_publish):
    # Lazy-start the state publisher.
    if self.st is None:
      self.st = StateThread(self)
      self.st.start()

  def cmd_abs(self, msg):
    bool2onoff={True:'on', False:'off'}
    if not self.twist_timeout:
        self.twist_timeout = True
        self.cmd_twist(Twist(),reset_timeout=False)
    now = rospy.Time.now()
    if (self.max_command_rate>0.0) and ((now - self.last_cmd).to_sec() < 1.0/self.max_command_rate):
        return
    self.last_cmd = now
    conn = httplib.HTTPConnection(self.hostname)
    # Flip pan orient if the camera is mounted backwards and facing down
    if self.flip:
    	msg.pan = 180 - msg.pan
        if msg.pan > 180:
	    msg.pan -= 360
        if msg.pan < -180:
	    msg.pan += 360
    params = {}
    if msg.fields & msg.SELECT_PAN:
        params['pan'] = msg.pan
    if msg.fields & msg.SELECT_TILT:
        params['tilt'] = msg.tilt
    if msg.fields & msg.SELECT_ZOOM:
        params['zoom'] = msg.zoom
    if msg.fields & msg.SELECT_BRIGHTNESS:
        params['brightness'] = msg.brightness
    if msg.fields & msg.SELECT_IRFILTER:
        params['irfilter'] = bool2onoff[msg.irfilter]
    if msg.fields & msg.SELECT_AUTOFOCUS:
        params['autofocus'] = bool2onoff[msg.autofocus]
    if not msg.autofocus and msg.fields & msg.SELECT_FOCUS:
          params['focus'] = msg.focus
    if msg.fields & msg.SELECT_IRIS:
      rospy.logwarn("Iris value is read-only.")
    conn.request("GET", "/axis-cgi/com/ptz.cgi?%s" % urllib.urlencode(params))

  def cmd_twist(self, msg, reset_timeout=True):
    if reset_timeout:
        self.twist_timeout = False
    self.last_request = rospy.Time.now()
    connZ = httplib.HTTPConnection(self.hostname)
    connPT = httplib.HTTPConnection(self.hostname)
    pan = int(msg.angular.z * 180./math.pi)
    tilt = int(msg.angular.y * 180./math.pi)
    zoom = int(msg.linear.x)
    if pan>100:
        pan=100
    if pan<-100:
        pan=-100
    if tilt>100:
        tilt=100
    if tilt<-100:
        tilt=-100
    if zoom>100:
        zoom=100
    if zoom<-100:
        zoom=-100
    # Flip pan orient if the camera is mounted backwards and facing down
    if self.flip:
        pan = -pan
        tilt = -tilt
    connZ.request("GET", "/axis-cgi/com/ptz.cgi?continuouszoommove=%d" % (zoom))
    connPT.request("GET", "/axis-cgi/com/ptz.cgi?continuouspantiltmove=%d,%d" % (pan,tilt))


def main():
  rospy.init_node("axis_twist")

  arg_defaults = {
      'hostname': '192.168.0.90',
      'username': '',
      'password': '',
      'delay': 0.0,
      'max_command_rate': 0.0,
      'flip': True,
      }
  args = {}
  for name in arg_defaults:
    args[name] = rospy.get_param(rospy.search_param(name), arg_defaults[name])
  rospy.sleep(args['delay'])
  rospy.loginfo("Starting AXIS PTZ control")

  AxisPTZ(**args)
  rospy.spin()


if __name__ == "__main__":
  main()
