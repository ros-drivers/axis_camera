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
from math import nan as NaN
from math import radians as deg2rad
import threading
import time

from ptz_action_server_msgs.action import PtzMove
from ptz_action_server_msgs.msg import PtzState, Ptz
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
import requests
from sensor_msgs.msg import JointState, Joy


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


def rescale(x, old_min, old_max, new_min, new_max, clamp=True):
    """
    Do a linear rescale of a value so it lies within a new range.

    @param x  The value to rescale
    @param old_min  The minimum value of the original range of x
    @param old_max  The maximum value of the original range of x
    @param new_min  The new range's minimum
    @param new_max  The new range's maximum
    @param clamp  If True and x is outside the original range,
                  restrict it to lie inside the new range
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

        # PTZ parameters
        self.min_pan = self.axis.get_parameter('min_pan').value
        self.max_pan = self.axis.get_parameter('max_pan').value
        self.min_tilt = self.axis.get_parameter('min_tilt').value
        self.max_tilt = self.axis.get_parameter('max_tilt').value
        self.min_zoom = self.axis.get_parameter('min_zoom').value
        self.max_zoom = self.axis.get_parameter('max_zoom').value
        self.max_pan_speed = self.axis.get_parameter('max_pan_speed').value
        self.max_tilt_speed = self.axis.get_parameter('max_tilt_speed').value

        # PS4 controller parameters
        self.button_enable_pan_tilt = self.axis.get_parameter('button_enable_pan_tilt').value
        self.button_enable_zoom = self.axis.get_parameter('button_enable_zoom').value
        self.axis_pan = self.axis.get_parameter('axis_pan').value
        self.axis_tilt = self.axis.get_parameter('axis_tilt').value
        self.invert_tilt = self.axis.get_parameter('invert_tilt').value
        self.axis_zoom_in = self.axis.get_parameter('axis_zoom_in').value
        self.axis_zoom_out = self.axis.get_parameter('axis_zoom_out').value
        self.zoom_in_offset = self.axis.get_parameter('zoom_in_offset').value
        self.zoom_out_offset = self.axis.get_parameter('zoom_out_offset').value
        self.zoom_in_scale = self.axis.get_parameter('zoom_in_scale').value
        self.zoom_out_scale = self.axis.get_parameter('zoom_out_scale').value
        self.scale_pan = self.axis.get_parameter('scale_pan').value
        self.scale_tilt = self.axis.get_parameter('scale_tilt').value
        self.scale_zoom = self.axis.get_parameter('scale_zoom').value

        # The last-sent PTZ velocity control from the game controller
        self.last_teleop_velocity = PtzMove.Goal()
        self.last_cmd_velocity = PtzMove.Goal()

        self.set_ptz_absolute_srv = ActionServer(
            self.axis,
            PtzMove,
            'move_ptz/position_abs',
            self.move_ptz_abs_cb,
            cancel_callback=self.cancel_ptz_abs_cb,
            callback_group=ReentrantCallbackGroup()
        )

        self.set_ptz_relative_srv = ActionServer(
            self.axis,
            PtzMove,
            'move_ptz/position_rel',
            self.move_ptz_rel_cb,
            cancel_callback=self.cancel_ptz_rel_cb,
            callback_group=ReentrantCallbackGroup()
        )

        self.set_ptz_velocity_srv = ActionServer(
            self.axis,
            PtzMove,
            'move_ptz/velocity',
            self.move_ptz_vel_cb,
            cancel_callback=self.cancel_ptz_vel_cb,
            callback_group=ReentrantCallbackGroup()
        )

        if teleop:
            self.joy_sub = self.axis.create_subscription(
                Joy,
                "joy",
                self.joy_cb,
                10
            )

        self.cmd_vel_sub = self.axis.create_subscription(Ptz, 'cmd/velocity', self.cmd_velocity_cb, 1)
        self.joint_state_pub = self.axis.create_publisher(JointState, 'joint_states', 1)
        self.ptz_state_pub = self.axis.create_publisher(PtzState, 'ptz_state', 1)
        self.ptz_state = PtzState()
        self.joint_state_thread = threading.Thread(target=self.publish_joint_states)
        self.joint_state_thread.start()

        # center the camera on startup
        self.send_position(0, 0, 1)

    def publish_joint_states(self):
        rate = self.axis.create_rate(1)

        joints = JointState()
        joints.name = [
            f"{self.axis.tf_prefix}_pan_joint",
            f"{self.axis.tf_prefix}_tilt_joint"
        ]
        joints.velocity = [0.0, 0.0]
        joints.effort = [0.0, 0.0]

        while rclpy.ok():
            rate.sleep()

            (pan, tilt, zoom) = self.current_ptz()
            joints.header.stamp = self.axis.get_clock().now().to_msg()
            joints.position = [pan, tilt]

            self.ptz_state.pan = pan
            self.ptz_state.tilt = tilt
            self.ptz_state.zoom = zoom

            self.joint_state_pub.publish(joints)
            self.ptz_state_pub.publish(self.ptz_state)

    def current_ptz(self):
        """
        Get the current PTZ position of the camera.

        @return A tuple of the form (pan, tilt, zoom) where pan & tilt are in radians
                and zoom is in "x". Values are NaN if there is an error reading the
                values from the camera
        """
        d = self.axis.queryCameraPosition()

        pan = deg2rad(d['pan']) if 'pan' in d else NaN
        tilt = deg2rad(d['tilt']) if 'tilt' in d else NaN
        zoom = rescale(d['zoom'], 1, 9999, self.min_zoom, self.max_zoom) if 'zoom' in d else NaN

        return (pan, tilt, zoom)

    def send_position(self, cmd_pan, cmd_tilt, cmd_zoom):
        """
        Send a goal position to the camera.

        @param cmd_pan  The commanded pan (degrees)
        @param cmd_tilt  The commanded tilt (degrees)
        @param cmd_zoom  The commanded zoom (1-9999)

        @return True if the command was successfully sent
        """
        cmd_string = (
            f'/axis-cgi/com/ptz.cgi?pan={int(cmd_pan)}'
            f'&tilt={int(cmd_tilt)}'
            f'&zoom={int(cmd_zoom)}'
        )
        url = f'http://{self.axis.hostname}:{self.axis.http_port}/{cmd_string}'
        resp = requests.get(
            url,
            auth=self.axis.http_auth,
            timeout=self.axis.http_timeout,
            headers=self.axis.http_headers
        )
        if not self.axis.is_success(resp):
            self.axis.get_logger().warning(
                f'Failed to command absolute PTZ position: {resp}'
            )
        return self.axis.is_success(resp)

    def wait_for_position(self, goal_handle, cmd_pan, cmd_tilt, cmd_zoom):
        """
        Wait for the camera to reach its goal.

        @param goal_handle  The Ptz.action goal we're processing
        @param cmd_pan  The commanded pan (degrees)
        @param cmd_tilt  The commanded tilt (degrees)
        @param cmd_zoom  The commanded zoom (1-9999)

        @return True if we reached the goal, otherwise False
        """
        if not self.send_position(cmd_pan, cmd_tilt, cmd_zoom):
            # Error commanding the camera; abort the action
            goal_handle.abort()
            return False

        fb = PtzMove.Feedback()
        reached_goal = False
        prev_pan = NaN
        prev_tilt = NaN
        prev_zoom = NaN

        # Convert back to standard units now that we've sent the http request
        cmd_pan = deg2rad(cmd_pan)
        cmd_tilt = deg2rad(cmd_tilt)
        cmd_zoom = rescale(cmd_zoom, 1, 9999, self.min_zoom, self.max_zoom)

        while goal_handle and not reached_goal and not goal_handle.is_cancel_requested \
                and goal_handle.is_active:
            time.sleep(1)

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

            fb.ptz_remaining.pan = float(abs(cmd_pan - pan))
            fb.ptz_remaining.tilt = float(abs(cmd_tilt - tilt))
            fb.ptz_remaining.zoom = float(abs(cmd_zoom - zoom))

            goal_handle.publish_feedback(fb)

        if not goal_handle.is_active:
            return False
        if goal_handle.is_cancel_requested:
            goal_handle.cancel()
            return False
        return True

    def send_velocity_command(self, cmd_pan, cmd_tilt, cmd_zoom):
        """
        Send a velocity command to the camera.

        @param cmd_pan  The pan speed as a percentage of maximum [-100, 100]
        @param cmd_tilt  The tilt speed as a percentage of maximum [-100, 100]
        @param cmd_zoom  The zoom speed as a percentage of maximum [-100, 100]

        @return True if the command was sent successfully
        """
        cmd_string = (
            f'/axis-cgi/com/ptz.cgi?continuouspantiltmove={int(cmd_pan)},'
            f'{int(cmd_tilt)}&continuouszoommove={int(cmd_zoom)}'
        )
        url = f'http://{self.axis.hostname}:{self.axis.http_port}/{cmd_string}'
        resp = requests.get(
            url,
            auth=self.axis.http_auth,
            timeout=self.axis.http_timeout,
            headers=self.axis.http_headers
        )

        if not self.axis.is_success(resp):
            # Error commanding the camera; abort the action
            self.axis.get_logger().warning(
                f'Failed to command ptz velocity: {resp}'
            )
            return False
        return True

    def move_ptz_abs_cb(self, goal_handle):
        """
        Move the camera to an absolute PTZ position, relative to its base link.

        @param goal_handle
        """
        cmd_pan = round(rad2deg(clamp(goal_handle.request.ptz.pan, self.min_pan, self.max_pan)))
        cmd_tilt = round(rad2deg(clamp(goal_handle.request.ptz.tilt, self.min_tilt, self.max_tilt)))
        cmd_zoom = round(
            # Axis uses 1-9999 for internal zoom levels
            rescale(goal_handle.request.ptz.zoom, self.min_zoom, self.max_zoom, 1, 9999)
        )

        self.ptz_state.mode = PtzState.MODE_POSITION
        reached_goal = self.wait_for_position(goal_handle, cmd_pan, cmd_tilt, cmd_zoom)
        self.ptz_state.mode = PtzState.MODE_IDLE

        result = PtzMove.Result()
        result.success = reached_goal
        if reached_goal:
            goal_handle.succeed()

        return result

    def move_ptz_rel_cb(self, goal_handle):
        """
        Move the camera to a new PTZ position, relative to its current state.

        @param goal_handle
        """
        (current_pan, current_tilt, current_zoom) = self.current_ptz()

        cmd_pan = round(
            rad2deg(clamp(current_pan + goal_handle.request.ptz.pan, self.min_pan, self.max_pan))
        )
        cmd_tilt = round(
            rad2deg(clamp(current_tilt + goal_handle.request.ptz.tilt, self.min_tilt, self.max_tilt))
        )
        cmd_zoom = round(
            rescale(current_zoom + goal_handle.request.ptz.zoom, self.min_zoom, self.max_zoom, 1, 9999)
        )

        self.ptz_state.mode = PtzState.MODE_POSITION
        reached_goal = self.wait_for_position(goal_handle, cmd_pan, cmd_tilt, cmd_zoom)
        self.ptz_state.mode = PtzState.MODE_IDLE

        result = PtzMove.Result()
        result.success = reached_goal
        if reached_goal:
            goal_handle.succeed()
        return result

    def move_ptz_vel_cb(self, goal_handle):
        """
        Move the camera using velocity control.

        @param goal_handle
        """
        cmd_pan = round(
            rescale(goal_handle.request.ptz.pan, -self.max_pan_speed, self.max_pan_speed, -100, 100)
        )
        cmd_tilt = round(
            rescale(goal_handle.request.ptz.tilt, -self.max_tilt_speed, self.max_tilt_speed, -100, 100)
        )
        cmd_zoom = round(
            rescale(goal_handle.request.ptz.zoom, -1, 1, -100, 100)
        )

        result = PtzMove.Result()
        result.success = True

        fb = PtzMove.Feedback()
        fb.ptz_remaining.pan = clamp(goal_handle.request.pan, -self.max_pan_speed, self.max_pan_speed)
        fb.ptz_remaining.tilt = clamp(goal_handle.request.tilt,
                                  -self.max_tilt_speed, self.max_tilt_speed)
        fb.ptz_remaining.zoom = clamp(goal_handle.request.zoom, -1.0, 1.0)

        self.ptz_state.mode = PtzState.MODE_VELOCITY
        if not self.send_velocity_command(cmd_pan, cmd_tilt, cmd_zoom):
            goal_handle.abort()
            result.success = False
        else:
            # Continuous control; the only way to stop is to cancel
            while not goal_handle.is_cancel_requested and goal_handle.is_active:
                time.sleep(1)
                goal_handle.publish_feedback(fb)

        # Command the camera to stop moving
        self.ptz_state.mode = PtzState.MODE_IDLE
        self.axis.get_logger().warning("Cancelling velocity action")
        self.send_velocity_command(0, 0, 0)
        goal_handle.abort()
        return result

    def cancel_ptz_abs_cb(self, cancel_request):
        return CancelResponse.ACCEPT

    def cancel_ptz_rel_cb(self, cancel_request):
        return CancelResponse.ACCEPT

    def cancel_ptz_vel_cb(self, cancel_request):
        return CancelResponse.ACCEPT

    def cmd_velocity_cb(self, msg):
        """
        Start velocity-controlling the camera using the joystick input.

        @param msg  The ptz_action_server_msgs/Ptz message to process
        """
        pan = msg.pan
        tilt = msg.tilt
        zoom = msg.zoom
        if (
            pan != self.last_cmd_velocity.ptz.pan or
            tilt != self.last_cmd_velocity.ptz.tilt or
            zoom != self.last_cmd_velocity.ptz.zoom
        ):
            self.last_cmd_velocity.ptz.pan = pan
            self.last_cmd_velocity.ptz.tilt = tilt
            self.last_cmd_velocity.ptz.zoom = zoom

            # rescale pan & tilt to be -100 to 100; zoom is already in that range
            pan = rescale(pan, -self.max_pan_speed, self.max_pan_speed, -100, 100)
            tilt = rescale(tilt, -self.max_tilt_speed, self.max_tilt_speed, -100, 100)

            if pan != 0 or tilt != 0 or zoom != 0:
                self.ptz_state.mode = PtzState.MODE_VELOCITY
            else:
                self.ptz_state.mode = PtzState.MODE_IDLE
            self.send_velocity_command(pan, tilt, zoom)

    def joy_cb(self, msg):
        """
        Start velocity-controlling the camera using the joystick input.

        @param msg  The sensor_msgs/Joy message to process
        """
        pan = 0
        tilt = 0
        zoom = 0
        if self.button_enable_pan_tilt < 0 or msg.buttons[self.button_enable_pan_tilt]:
            pan = msg.axes[self.axis_pan] * -self.scale_pan
            tilt = msg.axes[self.axis_tilt] * self.scale_tilt * (-1 if self.invert_tilt else 1)

        if self.button_enable_zoom < 0 or msg.buttons[self.button_enable_zoom]:
            zoom_in_amt = (msg.axes[self.axis_zoom_in] + self.zoom_in_offset) * self.zoom_in_scale
            zoom_out_amt = (
                msg.axes[self.axis_zoom_out] + self.zoom_out_offset
            ) * self.zoom_out_scale
            zoom = (zoom_in_amt + zoom_out_amt) * self.scale_zoom

        if (
            pan != self.last_teleop_velocity.ptz.pan or
            tilt != self.last_teleop_velocity.ptz.tilt or
            zoom != self.last_teleop_velocity.ptz.zoom
        ):
            self.last_teleop_velocity.ptz.pan = pan
            self.last_teleop_velocity.ptz.tilt = tilt
            self.last_teleop_velocity.ptz.zoom = zoom

            # rescale pan & tilt to be -100 to 100; zoom is already in that range
            pan = rescale(pan, -self.max_pan_speed, self.max_pan_speed, -100, 100)
            tilt = rescale(tilt, -self.max_tilt_speed, self.max_tilt_speed, -100, 100)

            if pan != 0 or tilt != 0 or zoom != 0:
                self.ptz_state.mode = PtzState.MODE_VELOCITY
            else:
                self.ptz_state.mode = PtzState.MODE_IDLE
            self.send_velocity_command(pan, tilt, zoom)
