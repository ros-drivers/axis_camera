cmake_minimum_required(VERSION 3.0.2)
project(axis_description)

find_package(catkin REQUIRED
  COMPONENTS
)

catkin_package()

install(DIRECTORY config meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
