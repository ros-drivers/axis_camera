#!/usr/bin/env python3

# Copyright 2024 Clearpath Robotics Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Clearpath Robotics Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from math import degrees as rad2deg
from math import radians as deg2rad

from ptz_action_server_msgs.action import Ptz
from sensor_msgs.msg import Joy

from rclpy.action import ActionServer


class AxisPtz:
    """Provides action server interfaces for controlling the pan, tilt, and zoom of supported PTZ devices."""

    def __init__(self, camera, teleop=False):
        """
        Create a PTZ action server node to control the given camera.

        @param camera  An Axis instance that controls the underlying camera
        @param teleop  If True, subscribe to /joy_teleop/joy
        """
        self.axis = camera

        self.set_ptz_absolute_srv = ActionServer(
            self.axis,
            Ptz,
            'move_ptz/position_abs',
            self.move_ptz_abs_cb
        )

        self.set_ptz_relative_srv = ActionServer(
            self.axis,
            Ptz,
            'move_ptz/position_rel',
            self.move_ptz_rel_cb
        )

        self.set_ptz_velocity_srv = ActionServer(
            self.axis,
            Ptz,
            'move_ptz/velocity',
            self.move_ptz_vel_cb
        )

        if teleop:
            self.joy_sub = self.axis.create_subscription(
                Joy,
                "joy_teleop/joy",
                self.joy_cb,
                10
            )

    def move_ptz_abs_cb(self, goal_handle):
        """
        Move the camera to an absolute PTZ position, relative to its base link.

        @param goal_handle
        """
        pass

    def move_ptz_rel_cb(self, goal_handle):
        """
        Move the camera to a new PTZ position, relative to its current state.

        @param goal_handle
        """
        pass

    def move_ptz_vel_cb(self, goal_handle):
        """
        Move the camera using velocity control.

        @param goal_handle
        """
        pass

    def joy_cb(self, msg):
        """
        Start velocity-controlling the camera using the joystick input.

        @param msg  The sensor_msgs/Joy message to process
        """
        pass