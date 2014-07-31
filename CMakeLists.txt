cmake_minimum_required(VERSION 2.8.3)
project(axis_camera)

find_package(catkin REQUIRED
  COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  message_generation
  std_msgs
)
catkin_python_setup()

# ROS message generation
add_message_files(FILES Axis.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)
generate_dynamic_reconfigure_options(cfg/PTZ.cfg)

catkin_package()

install(PROGRAMS
  nodes/axis.py
  nodes/axis_ptz.py
  nodes/publish_axis_tf.py
  nodes/teleop.py
  nodes/teleop_speed_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch tests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

