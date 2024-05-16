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
from math import isnan
from math import nan as NaN
from math import radians as deg2rad

from ptz_action_server_msgs.action import Ptz
import requests
from sensor_msgs.msg import Joy

from rclpy.action import ActionServer


def clamp(x, low=0, high=1):
    """
    Clamp a value to lie inside a desired range.

    @param x  The value to clamp
    @param low  The lower end of the range
    @param high  The upper end of the range
    """
    if x < low:
        return low
    if x > high:
        return high
    return x


def rescale(x, old_min, old_max, new_min, new_max, clamp=True):  # noqa: PLR0913 PLR0917
    """
    Do a linear rescale of a value so it lies within a new range.

    @param x  The value to rescale
    @param old_min  The minimum value of the original range of x
    @param old_max  The maximum value of the original range of x
    @param new_min  The new range's minimum
    @param new_max  The new range's maximum
    @param clamp  If True and x is outside the original range, restrict it to lie inside the new range
    """
    if clamp and x < old_min:
        return new_min
    if clamp and x > old_max:
        return new_max

    return (x - old_min) / (old_max - old_min) * (new_max - new_min) + new_min


class AxisPtz:
    """Provides action server interfaces for controlling PTZ of supported devices."""

    def __init__(self, camera, teleop=False):
        """
        Create a PTZ action server node to control the given camera.

        @param camera  An Axis instance that controls the underlying camera
        @param teleop  If True, subscribe to /joy_teleop/joy
        """
        self.axis = camera

        self.min_pan = self.axis.get_parameter('min_pan').value
        self.max_pan = self.axis.get_parameter('max_pan').value
        self.min_tilt = self.axis.get_parameter('min_tilt').value
        self.max_tilt = self.axis.get_parameter('max_tilt').value
        self.min_zoom = self.axis.get_parameter('min_zoom').value
        self.max_zoom = self.axis.get_parameter('max_zoom').value
        self.max_pan_speed = self.axis.get_parameter('max_pan_speed').value
        self.max_tilt_speed = self.axis.get_parameter('max_tilt_speed').value

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

    def current_ptz(self):
        """
        Get the current PTZ position of the camera.

        @return A tuple of the form (pan, tilt, zoom) where pan & tilt are in radians. Values are
                NaN if there is an error reading the values from the camera
        """
        d = self.axis.queryCameraPosition()

        pan = deg2rad(d['pan']) if 'pan' in d else NaN
        tilt = deg2rad(d['tilt']) if 'tilt' in d else NaN
        zoom = d['zoom'] if 'zoom' in d else NaN  # noqa: SIM401

        return (pan, tilt, zoom)

    def move_ptz_abs_cb(self, goal_handle):  # noqa: PLR0914
        """
        Move the camera to an absolute PTZ position, relative to its base link.

        @param goal_handle
        """
        rate = self.axis.create_rate(1)  # feedback at 1Hz

        cmd_pan = round(rad2deg(clamp(goal_handle.request.pan, self.min_pan, self.max_pan)))
        cmd_tilt = round(rad2deg(clamp(goal_handle.request.tilt, self.min_tilt, self.max_tilt)))
        cmd_zoom = round(
            # Axis uses 1-9999 for internal zoom levels
            rescale(goal_handle.request.zoom, self.min_zoom, self.max_zoom, 1, 9999)
        )

        cmd_string = f'/axis-cgi/com/ptz.cgi?pan={int(cmd_pan)}&tilt={int(cmd_tilt)}&zoom={int(cmd_zoom)}'  # noqa: E501
        url = f'http://{self.axis.hostname}:{self.axis.http_port}/{cmd_string}'
        resp = requests.get(
            url,
            auth=self.axis.http_auth,
            timeout=self.axis.http_timeout,
            headers=self.axis.http_headers
        )

        if resp.status_code != requests.status_codes.codes.ok:
            # Error commanding the camera; abort the action
            self.axis.get_logger().warning(
                f'Failed to command absolute PTZ position: {resp}'
            )
            goal_handle.abort()

        fb = Ptz.Feedback()
        reached_goal = False
        prev_pan = NaN
        prev_tilt = NaN
        prev_zoom = NaN
        while not reached_goal:
            rate.sleep()

            (pan, tilt, zoom) = self.current_ptz()

            if (
                pan - prev_pan == 0 and
                tilt - prev_tilt == 0 and
                zoom - prev_zoom == 0
            ):
                # We've stopped moving; assume we've reached the goal
                reached_goal = True

            prev_pan = pan
            prev_tilt = tilt
            prev_zoom = zoom

            fb.pan_remaining = deg2rad(abs(cmd_pan - pan))
            fb.tilt_remaining = deg2rad(abs(cmd_tilt - tilt))
            fb.zoom_remaining = rescale(abs(cmd_zoom - zoom), 1, 9999, self.min_zoom, self.max_zoom)

            goal_handle.publish_feedback(fb)

        result = Ptz.Result()
        result.success = True
        goal_handle.succeed()
        return result

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