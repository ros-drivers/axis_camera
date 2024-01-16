import rclpy
from rclpy.node import Node
from axis_camera_ros2.axis_camera import Axis

def updateArgs(arg_defaults):
    '''Look up parameters starting in the driver's private parameter space, but
    also searching outer namespaces
    '''
    args = {}
    for name, val in arg_defaults.items():
        full_name = rclpy.search_param(name)
        if full_name is None:
            args[name] = val
        else:
            args[name] = rclpy.get_param(full_name, val)
    # resolve frame_id with tf_prefix (unless already absolute)
    if args['frame_id'][0] != '/':        # not absolute?
        tf_prefix = rclpy.search_param('tf_prefix')
        prefix_val = ''
        if tf_prefix is not None:           # prefix defined?
            prefix_val = rclpy.get_param(tf_prefix)
            if prefix_val[0] != '/':          # prefix not absolute?
                prefix_val = '/' + prefix_val
        args['frame_id'] = prefix_val + '/' + args['frame_id']
    return(args)

def main(args=None):
    rclpy.init()

    #parameters = {
    #    'hostname': '',       # default IP address
    #    'username': '',               # default login name
    #    'password': '',
    #    'width': 640,
    #    'height': 480,
    #    'fps': 20,                         # frames per second (0 = camera default)
    #    'frame_id': 'axis_camera_link',
    #    'camera_info_url': '',
    #    'use_encrypted_password' : False,
    #    'camera' : 1,
    #    'ir': False,
    #    'defog': False,
    #    'wiper': False,
    #    'ptz': False }

    #args = updateArgs(parameters)

    node_name='axis_camera_node'

    axis_camera_node = Axis(node_name)

    rclpy.spin(axis_camera_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    axis_camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()