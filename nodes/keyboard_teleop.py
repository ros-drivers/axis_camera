#!/usr/bin/env python
"""
Keyboard teleoperation node for the Axis camera.

Use WSAD to control pan-tilt, and QE to control zoom. Shift increases speed.
Do not hold the buttons all the time, the commands need some time to get executed.
"""

import rospy

from axis_camera.msg import PTZ

import sys, select, termios, tty
from std_msgs.msg import Header

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   q  w  e
   a  s  d

q/e: zoom
a/d: pan
w/s: tilt
Shift modifier: faster

space: 0,0,0 position


CTRL-C to quit
"""

moveBindings = {
    'a': (-1, 0, 0),
    'd': (1, 0, 0),
    's': (0, -1, 0),
    'w': (0, 1, 0),
    'q': (0, 0, -100),
    'e': (0, 0, 100),
    'A': (-10, 0, 0),
    'D': (10, 0, 0),
    'S': (0, -10, 0),
    'W': (0, 10, 0),
    'Q': (0, 0, -1000),
    'E': (0, 0, 1000),
    ' ': None,
}


def get_key():
    """
    Read a key typed to the terminal.
    :return: The key that was typed.
    :rtype: basestring
    """
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    new_settings = termios.tcgetattr(sys.stdin)
    new_settings[3] = new_settings[3] & ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, new_settings)
    return key

if __name__ == "__main__":
    # save current terminal settings
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('axis_keyboard_teleop')

    absolutePtzPublisher = rospy.Publisher('axis/control/pan_tilt_zoom/absolute', PTZ, queue_size=10)
    relativePtzPublisher = rospy.Publisher('axis/control/pan_tilt_zoom/relative', PTZ, queue_size=10)

    print msg

    rate = rospy.Rate(1)

    try:
        while not rospy.is_shutdown():
            key = get_key()
            if key in moveBindings.keys():
                if moveBindings[key] is None:
                    absolutePtzPublisher.publish(PTZ(Header(), 0, 0, 0))
                else:
                    pan = moveBindings[key][0]
                    tilt = moveBindings[key][1]
                    zoom = moveBindings[key][2]

                    relativePtzPublisher.publish(PTZ(Header(), pan, tilt, zoom))

            elif key == '\x03':
                    break

            rate.sleep()
    except Exception as e:
        print e
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
