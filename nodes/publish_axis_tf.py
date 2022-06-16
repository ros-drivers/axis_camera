#!/usr/bin/env python3

import rospy
import math
from axis_camera.msg import Axis
import tf
from tf.transformations import quaternion_from_euler

base_name = "/kingfisher"
base_frame = "/kingfisher/base"
broadcaster = tf.TransformBroadcaster()

def axis_cb(data):
    global broadcaster, base_frame, base_name
    pan = math.pi + data.pan * math.pi / 180.
    tilt = -data.tilt * math.pi / 180.
    q = quaternion_from_euler(0,0,pan)
    now = rospy.Time.now()
    broadcaster.sendTransform((0,0,0),
            q,now,base_name+"/pan",base_frame)
    q = quaternion_from_euler(0,tilt,0)
    broadcaster.sendTransform((0,0,0),
            q,now,base_name+"/tilt",base_name+"/pan")


if __name__ == '__main__':
    try:
        rospy.init_node("axis_tf_broadcaster")
        base_frame = rospy.get_param("~base_frame",base_frame)
        base_name = rospy.get_param("~base_name",base_name)
        axis_sub = rospy.Subscriber("/axis/state",Axis,axis_cb)
        rospy.loginfo("Started Axis TF broadcaster")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
