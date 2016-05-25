cmake_minimum_required(VERSION 2.8.3)
project(axis_camera)

set(THIS_PACKAGE_MESSAGE_DEPS
  sensor_msgs
  std_msgs
)
# packages which are both build and run dependencies
set(THIS_PACKAGE_DEPS
  camera_info_manager_py
  diagnostic_aggregator
  diagnostic_msgs
  diagnostic_updater
  dynamic_reconfigure
  rospy
  sensor_msgs
  std_msgs
  tf
  ${THIS_PACKAGE_MESSAGE_DEPS}
)

find_package(catkin REQUIRED COMPONENTS message_generation xacro ${THIS_PACKAGE_DEPS})
catkin_python_setup()

# ROS message generation
add_message_files(FILES
  PTZ.msg
  PointInRectangle.msg
  Axis.msg # deprecated
)
add_service_files(FILES
  TakeSnapshot.srv
)
generate_messages(DEPENDENCIES ${THIS_PACKAGE_MESSAGE_DEPS})

generate_dynamic_reconfigure_options(
  cfg/Camera.cfg
  cfg/VideoStream.cfg
  cfg/PTZ.cfg # deprecated
)

# Generate the standalone URDF version.
xacro_add_xacro_file(${CMAKE_CURRENT_SOURCE_DIR}/urdf/axis_214_PTZ_standalone.xacro
                    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/axis_214_PTZ_standalone.urdf)
add_custom_target(axis_camera_media_files_urdf ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/urdf/axis_214_PTZ_standalone.urdf)

catkin_package(CATKIN_DEPENDS message_runtime ${THIS_PACKAGE_DEPS})

install(PROGRAMS
  nodes/axis.py
  nodes/axis_ptz.py
  nodes/teleop.py
  nodes/teleop_speed_control.py
  nodes/keyboard_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch tests meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES diag.yaml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
