cmake_minimum_required(VERSION 3.0.2)
project(axis_camera)

find_package(catkin REQUIRED
  COMPONENTS
)
catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  nodes/axis_camera_node
  nodes/teleop.py
  nodes/teleop_speed_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch tests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
