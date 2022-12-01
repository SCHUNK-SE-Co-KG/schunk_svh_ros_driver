cmake_minimum_required(VERSION 3.10)
project(schunk_svh_driver)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    combined_robot_hw
    controller_manager
    dynamic_reconfigure
    hardware_interface
    joint_limits_interface
    pluginlib
    roscpp
    rospy
    schunk_svh_description
    schunk_svh_msgs
    sensor_msgs
    std_msgs
)

## System dependencies are found with CMake's conventions
find_package(schunk_svh_library REQUIRED)

catkin_python_setup()

#dynamic reconfigure files
generate_dynamic_reconfigure_options(
  cfg/svh.cfg
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES svh_robot_hw svh_ros
  CATKIN_DEPENDS
    actionlib
    combined_robot_hw
    controller_manager
    dynamic_reconfigure
    hardware_interface
    joint_limits_interface
    pluginlib
    roscpp
    rospy
    schunk_svh_description
    schunk_svh_msgs
    sensor_msgs
    std_msgs
  DEPENDS
    schunk_svh_library
)

###########
## Build ##
###########

# mesh file generations
# iterate through all the stl files to:
#  1. convert to *.iv files
#  2. generate convex decomposed .stl


install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${driver_svh_INCLUDE_DIRS}
)

## ROSControl
add_library(svh_ros
  src/DynamicParameterClass.cpp
  src/ROSLogHandler.cpp
  src/SVHDiagnostics.cpp
  src/SVHWrapper.cpp
)

add_dependencies(svh_ros
  ${catkin_EXPORTED_TARGETS}
  ${schunk_svh_driver_EXPORTED_TARGETS}
)


## Specify libraries to link a library or executable target against
target_link_libraries(svh_ros
  ${catkin_LIBRARIES}
  Schunk::svh-library
)

add_library(svh_robot_hw
  src/SVHRosControlHWInterface.cpp
)
add_dependencies(svh_robot_hw
  svh_ros
  ${catkin_EXPORTED_TARGETS}
  ${schunk_svh_driver_EXPORTED_TARGETS}
)
target_link_libraries(svh_robot_hw
  svh_ros
  ${catkin_LIBRARIES}
)

add_executable(svh_ros_control_node
  src/ROSLogHandler.cpp
  src/svh_ros_control_node.cpp
)
add_dependencies(svh_ros_control_node
  svh_ros
  svh_robot_hw
  ${schunk_svh_driver_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(svh_ros_control_node
  svh_ros
  svh_robot_hw
  ${catkin_LIBRARIES}
)

## DynamicParametersTest ##

add_executable(svh_dynamic_parameter_test
  src/DynamicParameterClass.cpp
  src/DynamicParameterTest.cpp
  src/ROSLogHandler.cpp
)

add_dependencies(svh_dynamic_parameter_test
  ${schunk_svh_driver_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(svh_dynamic_parameter_test
 ${catkin_LIBRARIES}
 Schunk::svh-library
)


#############
## Install ##
#############

# Install scripts
install(PROGRAMS
  src/rqt_svh_reset_gui/svh_reset_gui.py
  udev/create_udev_rules
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install targets of libs and executables
install(
  TARGETS
    svh_robot_hw
    svh_ros
    svh_ros_control_node
    svh_dynamic_parameter_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install the headers
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install config files
install(DIRECTORY cfg etc resource udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install individual files
install(
  FILES
    plugin.xml
    robot_hw_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
