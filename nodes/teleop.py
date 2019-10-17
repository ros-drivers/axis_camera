#!/usr/bin/python

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from axis_camera.msg import Axis


class Teleop:
    def __init__(self):
        self.enable_button = rospy.get_param('~enable_button', 1)
        self.enable_turbo_button = rospy.get_param('~enable_turbo_button', 3)
        self.camera_reset_pose_1 = rospy.get_param('~camera_reset_pose_1', 4)
        self.camera_reset_pose_2 = rospy.get_param('~camera_reset_pose_2', 5)
        self.axis_pan = rospy.get_param('~axis_pan', 0)
        self.axis_tilt = rospy.get_param('~axis_tilt', 1)
        self.axis_zoom = rospy.get_param('~axis_zoom', 6)
        self.pub_time = rospy.Time.now()
        self.pub = rospy.Publisher('cmd', Axis, queue_size=1)
        self.state = Axis(pan=10)
        self.state.brightness = 5000
        self.pub.publish(self.state)

        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)
        # rospy.Subscriber("state", Axis, self.state_callback)

    def joy_callback(self, data):
        if data.buttons[self.enable_button] == 1 and (rospy.Time.now() - self.pub_time).to_sec() > 1.0 and (rospy.Time.now() - data.header.stamp).to_sec() < 0.2:

            self.state.pan = self.angle_wrap(self.state.pan - 10.0 *np.sign(data.axes[self.axis_pan]))  #rotate 10 degrees at least

            self.state.tilt = self.state.tilt + 5.0 * np.sign(data.axes[self.axis_tilt])  #tilt at least 5 degrees
            if self.state.tilt > 85: self.state.tilt = 85
            if self.state.tilt < 0: self.state.tilt = 0

            self.state.zoom = self.state.zoom + 50.0 * np.sign(data.axes[self.axis_zoom])
            if self.state.zoom > 1000: self.state.zoom = 1000
            if self.state.zoom < 0: self.state.zoom = 0

            self.pub.publish(self.state)
            self.pub_time = rospy.Time.now()
            rospy.sleep(1.0)

        if data.buttons[self.enable_turbo_button] == 1 and (rospy.Time.now() - self.pub_time).to_sec() > 1.0 and (rospy.Time.now() - data.header.stamp).to_sec() < 0.2:

            self.state.pan = self.angle_wrap(self.state.pan - 25.0 *np.sign(data.axes[self.axis_pan]))  #rotate 5 degrees at least

            self.state.tilt = self.state.tilt + 10.0 * np.sign(data.axes[self.axis_tilt])  #tilt at least 10 degrees
            if self.state.tilt > 85: self.state.tilt = 85
            if self.state.tilt < 0: self.state.tilt = 0

            self.state.zoom = self.state.zoom + 200.0 * np.sign(data.axes[self.axis_zoom])
            if self.state.zoom > 1000: self.state.zoom = 1000
            if self.state.zoom < 0: self.state.zoom = 0
            
            self.pub.publish(self.state)
            self.pub_time = rospy.Time.now()
            rospy.sleep(1.0)

        if data.buttons[self.camera_reset_pose_1] == 1 and data.buttons[self.camera_reset_pose_2] == 1 and (rospy.Time.now() - self.pub_time).to_sec() > 1.0 and (rospy.Time.now() - data.header.stamp).to_sec() < 0.2:
            
            self.state.pan = 10.0
            self.state.tilt = 0.0
            self.state.zoom = 0.0           
            self.pub.publish(self.state)
            self.pub_time = rospy.Time.now()
            rospy.sleep(1.0)

    def angle_wrap(self,angle,rad=False):

        PI = 3.1415
        if rad:
            wrapped = angle % (2.0*PI)
            if wrapped < 0.0:
                wrapped = 2.0*PI + wrapped

        else:

            wrapped = angle % 360.0
            if wrapped < 0.0:
                wrapped = 360.0 + wrapped

        return wrapped 


if __name__ == "__main__": 

    rospy.init_node('axis_ptz_teleop')

    try:
        Teleop()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()

