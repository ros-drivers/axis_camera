cmake_minimum_required(VERSION 2.8.3)
project(axis_camera)

find_package(catkin REQUIRED
  COMPONENTS
  camera_info_manager_py
  geometry_msgs
  message_generation
  rospy
  sensor_msgs
  tf
)
catkin_python_setup()

# ROS message generation
add_message_files(DIRECTORY msg FILES Axis.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS
  camera_info_manager_py
  geometry_msgs
  message_runtime
  sensor_msgs
  tf
)

install(PROGRAMS
  nodes/axis.py
  nodes/axis_ptz.py
  nodes/publish_axis_tf.py
  nodes/teleop.py
  nodes/teleop_speed_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
