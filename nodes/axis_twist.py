#!/usr/bin/env python
#
# Basic PTZ node, based on documentation here:
#   http://www.axis.com/files/manuals/vapix_ptz_45621_en_1112.pdf
#

import os, sys, string, time, getopt, threading
import httplib, urllib
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
      if not self.axis.timeout and ((rospy.Time.now() - self.axis.last_request).to_sec() > 1.0):
          self.axis.timeout = True
          self.axis.cmd(Twist(),reset_timeout=False)
      conn = httplib.HTTPConnection(self.axis.hostname)
      params = { 'query':'position' }
      conn.request("GET", "/axis-cgi/com/ptz.cgi?%s" % urllib.urlencode(params))
      response = conn.getresponse()
      if response.status == 200:
        body = response.read()
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
          msg.focus = float(params['focus'])
        msg.autofocus = (params['autofocus'] == 'on')
        self.axis.pub.publish(msg)
      r.sleep()
      

class AxisPTZ:
  def __init__(self, hostname, username, password, flip):
    self.hostname = hostname
    self.username = username
    self.password = password
    self.flip = flip

    self.st = None
    self.timeout = True
    self.pub = rospy.Publisher("state", Axis, self)
    self.sub = rospy.Subscriber("twist", Twist, self.cmd)
    self.cmd(Twist(),reset_timeout=False)

  def peer_subscribe(self, topic_name, topic_publish, peer_publish):
    # Lazy-start the state publisher.
    if self.st is None:
      self.st = StateThread(self)
      self.st.start()

  def cmd(self, msg, reset_timeout=True):
    if reset_timeout:
        self.timeout = False
    self.last_request = rospy.Time.now()
    conn = httplib.HTTPConnection(self.hostname)
    pan = int(msg.angular.z * 180./math.pi)
    tilt = int(msg.angular.y * 180./math.pi)
    if pan>100:
        pan=100
    if pan<-100:
        pan=-100
    if tilt>100:
        tilt=100
    if tilt<-100:
        tilt=-100
    # Flip pan orient if the camera is mounted backwards and facing down
    if self.flip:
        pan = -pan
        tilt = -tilt
    conn.request("GET", "/axis-cgi/com/ptz.cgi?continuouspantiltmove=%d,%d" % (pan,tilt))


def main():
  rospy.init_node("axis_twist")

  arg_defaults = {
      'hostname': '192.168.0.90',
      'username': '',
      'password': '',
      'flip': True,
      }
  args = {}
  for name in arg_defaults:
    args[name] = rospy.get_param(rospy.search_param(name), arg_defaults[name])

  AxisPTZ(**args)
  rospy.spin()


if __name__ == "__main__":
  main()
