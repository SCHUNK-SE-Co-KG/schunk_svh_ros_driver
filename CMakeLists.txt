cmake_minimum_required(VERSION 2.8.3)
project(schunk_svh_driver)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    combined_robot_hw
    controller_manager
    dynamic_reconfigure
    hardware_interface
    joint_limits_interface
    message_generation
    pluginlib
    roscpp
    rospy
    rqt_gui
    rqt_gui_py
    sensor_msgs
    std_msgs
    urdf
    xacro
)

## System dependencies are found with CMake's conventions
find_package(icl_comm)
find_package(icl_core)
find_package(driver_svh)

catkin_python_setup()

#dynamic reconfigure files
generate_dynamic_reconfigure_options(
  cfg/svh.cfg
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  SVHDiagnosticsFinger.msg
  SVHDiagnosticsToProtocol.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   HomeAll.srv
   HomeWithChannels.srv
   SetAllChannelForceLimits.srv
   SetChannelForceLimit.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  SVHDiagnostics.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
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
    actionlib_msgs
    combined_robot_hw
    controller_manager
    hardware_interface
    message_runtime
    pluginlib
    roscpp
    sensor_msgs
    std_msgs
    urdf
  DEPENDS
    driver_svh
    icl_comm
    icl_core
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
  ${icl_comm_INCLUDE_DIRS}
  ${icl_core_INCLUDE_DIRS}
  ${driver_svh_INCLUDE_DIRS}
)

# Additional compiler definitions
add_definitions(
  ${icl_comm_DEFINITIONS}
  ${icl_core_DEFINITIONS}
)

# Declare a cpp executable
add_executable(svh_controller
  src/DynamicParameterClass.cpp
  src/SVHNode.cpp
)

# Simple test programs
add_executable(svh_sin_test
  src/SVHSinTest.cpp
)

# Add cmake target dependencies of the executable/library
# as an example, message headers may need to be generated before nodes
add_dependencies(svh_controller
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(svh_controller
  ${catkin_LIBRARIES}
  ${icl_comm_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${driver_svh_LIBRARIES}
)


## Specify libraries to link a library or executable target against
target_link_libraries(svh_sin_test
  ${catkin_LIBRARIES}
  ${icl_comm_LIBRARIES}
  ${icl_core_LIBRARIES}
)

## ROSControl
add_library(svh_ros
  src/DynamicParameterClass.cpp
  src/SVHDiagnostics.cpp
  src/SVHWrapper.cpp
)
add_dependencies(svh_ros
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(svh_ros
  ${catkin_LIBRARIES}
  ${icl_comm_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${driver_svh_LIBRARIES}
)

add_library(svh_robot_hw
  src/SVHRosControlHWInterface.cpp
)
add_dependencies( svh_robot_hw
  svh_ros
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(svh_robot_hw
  svh_ros
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(svh_ros_control_node
  src/svh_ros_control_node.cpp
)
add_dependencies(svh_ros_control_node
  svh_ros
  svh_robot_hw
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
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
)

add_dependencies(svh_dynamic_parameter_test
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(svh_dynamic_parameter_test
 ${catkin_LIBRARIES}
 ${catkin_LIBRARIES}
 ${icl_comm_LIBRARIES}
 ${icl_core_LIBRARIES}
 ${driver_svh_LIBRARIES}
)


#############
## Install ##
#############
# Special python handling
install(PROGRAMS
  src/rqt_svh_reset_gui/svh_reset_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install targets of libs and executables
install(
  TARGETS
    svh_controller
    svh_robot_hw
    svh_ros
    svh_ros_control_node
    svh_sin_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install the headers
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#config files
install(DIRECTORY etc/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/etc
)

#udev rules
install(DIRECTORY udev/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/udev
)

# Individual files (Quick commands and the urdf.rviz)
install(
  FILES
    plugin.xml
    quick_commands
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES robot_hw_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install resources
install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Helper for installing the udev rules in an easy fashion and without any conflicts
install(PROGRAMS udev/create_udev_rules
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
