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


import rclpy
from rclpy.executors import MultiThreadedExecutor

from axis_camera.axis_camera import Axis


def updateArgs(arg_defaults):
    """
    Update args with parameters in outer namespaces.

    Look up parameters starting in the driver's private parameter space,
    but also searching outer namespaces.
    """
    args = {}
    for name, val in arg_defaults.items():
        full_name = rclpy.search_param(name)
        if full_name is None:
            args[name] = val
        else:
            args[name] = rclpy.get_param(full_name, val)
    # resolve frame_id with tf_prefix (unless already absolute)
    if args['frame_id'][0] != '/':  # not absolute?
        tf_prefix = rclpy.search_param('tf_prefix')
        prefix_val = ''
        if tf_prefix is not None:  # prefix defined?
            prefix_val = rclpy.get_param(tf_prefix)
            if prefix_val[0] != '/':  # prefix not absolute?
                prefix_val = '/' + prefix_val
        args['frame_id'] = prefix_val + '/' + args['frame_id']
    return args


def main(args=None):
    rclpy.init()

    # parameters = {
    #    'hostname': '192.168.0.90',        # default IP address
    #    'http_port': 80,                   # default HTTP port
    #    'username': 'root',                # default login name
    #    'password': '',
    #    'width': 640,                      # frame width (pixels)
    #    'height': 480,                     # frame height (pixels)
    #    'fps': 20,                         # frames per second (0 = camera default)
    #    'tf_prefix': 'axis',               # sensor & joint frame prefix
    #    'camera_info_url': '',
    #    'use_encrypted_password' : False,
    #    'camera' : 1,
    #    'ir': False,
    #    'defog': False,
    #    'wiper': False,
    #    'ptz': False }

    # args = updateArgs(parameters)

    node_name = 'axis_camera_node'
    executor = MultiThreadedExecutor()
    axis_camera_node = Axis(node_name)
    rclpy.spin(axis_camera_node, executor=executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    axis_camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
