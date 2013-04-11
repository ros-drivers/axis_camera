#!/usr/bin/env python
#
# Axis camera image driver. Based on:
# https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/sandbox/axis_camera/axis.py
#

import os, sys, string, time
import urllib2

import roslib; roslib.load_manifest('axis_camera') 
import rospy 

from sensor_msgs.msg import CompressedImage, CameraInfo
from axis_camera.srv import *

import threading
import camera_info_manager

class StreamThread(threading.Thread):
  def __init__(self, axis):
    threading.Thread.__init__(self)

    self.axis = axis
    self.daemon = True

  def run(self):
    while True:
      try:
        rospy.loginfo("Connecting")
        self.stream()
      except:
        import traceback
        rospy.logerr(traceback.format_exc())
      time.sleep(1)

  def stream(self):
    url = 'http://%s/mjpg/video.mjpg' % self.axis.hostname
    url = url + "?fps=%d&resolultion=%dx%d" % (self.axis.fps,self.axis.width, self.axis.height)

    rospy.logdebug('opening ' + str(self.axis))

    # only try to authenticate if user/pass configured
    if self.axis.password != '' and self.axis.username != '':
      # create a password manager
      password_mgr = urllib2.HTTPPasswordMgrWithDefaultRealm()

      # Add the username and password, use default realm.
      top_level_url = "http://" + self.axis.hostname
      password_mgr.add_password(None, top_level_url,
                                self.axis.username,
                                self.axis.password)
      handler = urllib2.HTTPBasicAuthHandler(password_mgr)

      # create "opener" (OpenerDirector instance)
      opener = urllib2.build_opener(handler)

      # ...and install it globally so it can be used with urlopen.
      urllib2.install_opener(opener)

    fp = urllib2.urlopen(url)

    self.axis.reconnect = False
    while not self.axis.reconnect:
      boundary = fp.readline()

      header = {}
      while not self.axis.reconnect:
        line = fp.readline()
        if line == "\r\n": break
        line = line.strip()

        parts = line.split(": ", 1)
        header[parts[0]] = parts[1]

      content_length = int(header['Content-Length'])

      img = fp.read(content_length)
      line = fp.readline()
      
      msg = CompressedImage()
      msg.header.stamp = rospy.Time.now()
      msg.header.frame_id = self.axis.frame_id
      msg.format = "jpeg"
      msg.data = img

      self.axis.pub.publish(msg)

      try:
        cimsg = self.axis.cinfo.getCameraInfo()
        cimsg.header.stamp = msg.header.stamp
        cimsg.header.frame_id = self.axis.frame_id
        cimsg.width = self.axis.width
        cimsg.height = self.axis.height
        self.axis.caminfo_pub.publish(cimsg)
      except camera_info_manager.CameraInfoMissingError:
        # Ignore missing camera info
        pass

class Axis:
  def __init__(self, hostname, username, password,
               fps, width, height, frame_id, camera_info_url,delay):
    self.hostname = hostname
    self.username = username
    self.password = password
    self.fps = fps
    self.width = width
    self.height = height
    self.frame_id = frame_id
    self.camera_info_url = camera_info_url
    self.reconnect = False

    # generate a valid camera name based on the hostname
    self.cname = camera_info_manager.genCameraName(self.hostname)
    self.cinfo = camera_info_manager.CameraInfoManager(cname = self.cname,
                                                       url = self.camera_info_url)
    self.cinfo.loadCameraInfo()         # required before getCameraInfo()
    self.st = None
    self.pub = rospy.Publisher("image_raw/compressed", CompressedImage, self)
    self.caminfo_pub = rospy.Publisher("camera_info", CameraInfo, self)
    self.fps_srv = rospy.Service('set_parameters', SetParameters, self.handle_param_req)

  def handle_param_req(self,req):
      if req.fps >= 0:
          self.fps = req.fps
      if req.width > 0:
          self.width = req.width
      if req.height > 0:
          self.height = req.height
      self.reconnect = True
      return SetParametersResponse(self.fps,self.width,self.height)


  def __str__(self):
    """Return string representation."""
    return(self.hostname + ',' + self.username + ',' + self.password +
           '(' + str(self.width) + 'x' + str(self.height) + ')')

  def peer_subscribe(self, topic_name, topic_publish, peer_publish):
    # Lazy-start the image-publisher.
    if self.st is None:
      self.st = StreamThread(self)
      self.st.start()


def main():
  rospy.init_node("axis_driver")

  arg_defaults = {
      'hostname': '192.168.0.90',       # default IP address
      'username': 'root',               # default login name
      'password': '',
      'fps': 0,
      'width': 640,
      'height': 480,
      'delay': 0.0,
      'frame_id': 'axis_camera',
      'camera_info_url': ''}

  # Look up parameters starting in the driver's private parameter
  # space, but also searching outer namespaces.  Defining them in a
  # higher namespace allows the axis_ptz.py script to share parameters
  # with the driver.
  args = {}
  for name, val in arg_defaults.iteritems():
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

  rospy.sleep(args['delay'])

  Axis(**args)
  rospy.spin()


if __name__ == "__main__":
  main()
