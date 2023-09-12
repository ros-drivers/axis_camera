cmake_minimum_required(VERSION 3.0.2)
project(axis_msgs)

find_package(catkin REQUIRED
  COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

# ROS message generation
add_message_files(FILES
  Axis.msg
  Ptz.msg
)
add_service_files(FILES
  SetFloat.srv
)

generate_messages(DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package()

install(DIRECTORY msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
