cmake_minimum_required(VERSION 3.0.2)
project(axis_camera)

find_package(catkin REQUIRED
  COMPONENTS
  dynamic_reconfigure
)
catkin_python_setup()

# ROS message generation
generate_dynamic_reconfigure_options(cfg/PTZ.cfg)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/axis.py
  nodes/axis_ptz.py
  nodes/teleop.py
  nodes/teleop_speed_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch tests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
