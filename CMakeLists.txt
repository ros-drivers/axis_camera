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

<<<<<<< HEAD
install(PROGRAMS
  nodes/axis.py
  nodes/axis_ptz.py
  nodes/publish_axis_tf.py
  nodes/teleop.py
  nodes/teleop_speed_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
=======
#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_make_distribution(0.0.2)
>>>>>>> a0c9da512fdea3ab1d447d7d3f2e3139d6858886
