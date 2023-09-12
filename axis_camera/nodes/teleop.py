#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from axis_camera.msg import Axis


class Teleop:
    def __init__(self):
        rospy.init_node('axis_ptz_teleop')
        self.enable_button = rospy.get_param('~enable_button', 1)
        self.axis_pan = rospy.get_param('~axis_pan', 0)
        self.axis_tilt = rospy.get_param('~axis_tilt', 1)
        self.state = Axis(pan=220)
        self.joy = None

        self.pub = rospy.Publisher('cmd', Axis)
        rospy.Subscriber("joy", Joy, self.joy_callback)
        # rospy.Subscriber("state", Axis, self.state_callback)

    def spin(self):
        self.state.brightness = 5000
        self.pub.publish(self.state)
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.joy != None and self.joy.buttons[self.enable_button] == 1:
                #and (rospy.Time.now() - self.joy.header.stamp).to_sec() < 0.2:
                self.state.pan += self.joy.axes[self.axis_pan]*5
                self.state.tilt += self.joy.axes[self.axis_tilt]*5
                if self.state.tilt > 85: self.state.tilt = 85
                if self.state.tilt < 0: self.state.tilt = 0
                self.pub.publish(self.state)
            r.sleep()

    def joy_callback(self, data):
        self.joy = data


if __name__ == "__main__": Teleop().spin()
