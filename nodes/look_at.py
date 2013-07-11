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
        self.camera_frame = rospy.get_param('~camera_frame', "/kingfisher/axis")
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
        rospy.loginfo("Request to look at %.2f %.2f %.2f in %s" % (data.point.x,data.point.y,data.point.z,data.header.frame_id))
        try:
            self.listener.waitForTransform(self.camera_frame,
                    data.header.frame_id, data.header.stamp, rospy.Duration(1.0))
            P = self.listener.transformPoint(self.camera_frame,data).point
            rospy.loginfo("in local frame %s: %.2f %.2f %.2f" % (self.camera_frame,P.x,P.y,P.z))
            self.state.pan = (math.atan2(-P.y,P.x)+math.pi) * 180.0/math.pi
            self.state.tilt = math.atan2(-P.z, math.hypot(P.x,P.y)) * 180.0/math.pi
            rospy.loginfo("Looking at %.2f %.2f" % (self.state.pan, self.state.tilt))
            self.pub.publish(self.state)
        except Exception,e:
            rospy.loginfo("Exception while converting point frame: " + str(e))
 


if __name__ == "__main__": LookAt().spin()
