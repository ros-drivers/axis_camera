cmake_minimum_required(VERSION 2.4.6)
project(axis_camera)

if(USE_ROSBUILD)
    include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

    # Set the build type.  Options are:
    #  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
    #  Debug          : w/ debug symbols, w/o optimization
    #  Release        : w/o debug symbols, w/ optimization
    #  RelWithDebInfo : w/ debug symbols, w/ optimization
    #  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
    #set(ROS_BUILD_TYPE RelWithDebInfo)

    rosbuild_init()

    #set the default path for built executables to the "bin" directory
    #set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
    #set the default path for built libraries to the "lib" directory
    #set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
    rosbuild_find_ros_package(dynamic_reconfigure)
    include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
    gencfg()


    #uncomment if you have defined messages
    rosbuild_genmsg()
    #uncomment if you have defined services
    rosbuild_gensrv()

    #common commands for building c++ executables and libraries
    #rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
    #target_link_libraries(${PROJECT_NAME} another_library)
    #rosbuild_add_boost_directories()
    #rosbuild_link_boost(${PROJECT_NAME} thread)
    rosbuild_add_executable(republish src/republish.cpp)
    #target_link_libraries(example ${PROJECT_NAME})
else()

    find_package(catkin REQUIRED
      COMPONENTS
      camera_info_manager_py
      geometry_msgs
      message_generation
      rospy
      sensor_msgs
      tf
      cv_bridge
      dynamic_reconfigure
    )
    find_package(OpenCV REQUIRED)
    catkin_python_setup()

    # ROS message generation
    add_service_files(DIRECTORY srv FILES SetParameters.srv)

    add_message_files(DIRECTORY msg FILES Axis.msg)
    generate_messages(DEPENDENCIES geometry_msgs std_msgs)

    generate_dynamic_reconfigure_options(
      cfg/PTZ.cfg
    )

    catkin_package(
      CATKIN_DEPENDS
      camera_info_manager_py
      geometry_msgs
      message_runtime
      sensor_msgs
      cv_bridge
      tf
    )
    add_executable(republish src/republish.cpp)
    target_link_libraries(republish ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

    install(PROGRAMS
      nodes/axis.py
      nodes/axis_ptz.py
      nodes/publish_axis_tf.py
      nodes/teleop.py
      nodes/teleop_speed_control.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

    install(DIRECTORY
      cfg
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()
