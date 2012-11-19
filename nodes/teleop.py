#!/usr/bin/python

import roslib; roslib.load_manifest('clearpath_teleop')
import rospy

from sensor_msgs.msg import Joy
from axis_camera.msg import Axis


class Teleop:
    def __init__(self):
        rospy.init_node('axis_ptz_teleop')
        self.enable_button = rospy.get_param('~enable_button', 1)
        self.cmd = 0;

        self.pub = rospy.Publisher('cmd', Axis)
        rospy.Subscriber("joy", Joy, self.joy_callback)
        rospy.Subscriber("state", Axis, self.state_callback)
        
    def spin(self):
        rospy.spin()

    def state_callback(self, data):
        self.cmd = data

    def joy_callback(self, data):
        if data.buttons[self.enable_button] == 1:
            self.cmd.pan += data.axes[1]
            self.cmd.tilt += data.axes[0]
            self.pub.publish(self.cmd)


if __name__ == "__main__": Teleop().spin()
