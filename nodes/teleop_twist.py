#!/usr/bin/python

import roslib; roslib.load_manifest('axis_camera')
import rospy

import math
from sensor_msgs.msg import Joy
from axis_camera.msg import Axis
from geometry_msgs.msg import Twist


class Teleop:
    def __init__(self):
        rospy.init_node('axis_twist_teleop')
        self.enable_button = rospy.get_param('~enable_button', 1)
        self.zero_button = rospy.get_param('~zero_button', 2)
        self.scale_pan = rospy.get_param('~scale_pan_deg', 10)
        self.scale_tilt = rospy.get_param('~scale_tilt_deg', 10)
        self.state = Axis(pan=180,tilt=0,zoom=1)
        self.joy = None

        self.cmd_pub = rospy.Publisher('cmd', Axis)
        self.twist_pub = rospy.Publisher('twist', Twist)
        rospy.Subscriber("/joy", Joy, self.joy_callback)
        # rospy.Subscriber("state", Axis, self.state_callback)
        
    def spin(self):
        twist = Twist()
        self.cmd_pub.publish(self.state)
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.joy != None and self.joy.buttons[self.enable_button] == 1:
                twist.angular.z = self.joy.axes[0]*self.scale_pan*math.pi/180.
                twist.angular.y = -self.joy.axes[1]*self.scale_tilt*math.pi/180.
                self.twist_pub.publish(twist)
            if self.joy != None and self.joy.buttons[self.zero_button] == 1:
                self.cmd_pub.publish(self.state)
            r.sleep()

    def joy_callback(self, data):
        self.joy = data


if __name__ == "__main__": Teleop().spin()
