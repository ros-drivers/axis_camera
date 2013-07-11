#!/usr/bin/python

import roslib; roslib.load_manifest('axis_camera')
import rospy
import math

from geometry_msgs.msg import PointStamped


class TestLookAt:
    def __init__(self):
        rospy.init_node('axis_ptz_test_look_at')
        self.pub = rospy.Publisher('/axis/target', PointStamped)
        
    def spin(self):
        P = PointStamped()
        self.pub.publish(P)
        rospy.sleep(1.0)
        for i in range(0,12):
            if rospy.is_shutdown():
                break
            P = PointStamped()
            P.header.stamp = rospy.Time.now()
            P.header.frame_id = "/kingfisher/laser"
            P.point.x = 3.0 * math.cos(i*math.pi/6.)
            P.point.y = 3.0 * math.sin(i*math.pi/6.)
            P.point.z = 0.0
            rospy.loginfo("Axis: Looking at \n" + str(P.point))
            self.pub.publish(P)
            rospy.sleep(5.0)



if __name__ == "__main__": TestLookAt().spin()
