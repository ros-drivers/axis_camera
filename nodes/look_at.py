#!/usr/bin/python

import roslib; roslib.load_manifest('axis_camera')
import rospy
import math
import tf

from geometry_msgs.msg import PointStamped
from axis_camera.msg import Axis


class LookAt:
    def __init__(self):
        rospy.init_node('axis_ptz_look_at')
        self.camera_frame = rospy.get_param('~camera_frame', "/axis/tilt")
        self.state = Axis()
        self.listener = tf.TransformListener()

        self.pub = rospy.Publisher('/axis/cmd', Axis)
        rospy.Subscriber("/axis/target", PointStamped, self.target_callback)
        rospy.Subscriber("/axis/state", Axis, self.state_callback)
        rospy.loginfo("Axis LookAt: ready for commands")
        
    def spin(self):
        rospy.spin()

    def state_callback(self, data):
        self.state = data

    def target_callback(self, data):
        self.listener.waitForTransform(self.camera_frame,
                data.header.frame_id, data.header.stamp, rospy.Duration(1.0))
        ((x,y,z),rot) = self.listener.lookupTransform(self.camera_frame,
                data.header.frame_id, data.header.stamp)
        self.state.pan = ((math.atan2(y,x)+math.pi) * 180.0/math.pi) % 360.0
        self.state.tilt = math.atan2(-z, math.hypot(x,y)) * 180.0/math.pi
        self.pub(self.state)
 


if __name__ == "__main__": LookAt().spin()
