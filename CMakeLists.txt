cmake_minimum_required(VERSION 2.8.3)
project(schunk_svh_driver)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    dynamic_reconfigure
    rospy
    rqt_gui
    rqt_gui_py
    xacro
    hardware_interface
    joint_limits_interface
    controller_manager
    urdf
    message_generation
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

## Generate services in the 'srv' folder
add_service_files(
   FILES
   HomeAll.srv
   HomeWithChannels.srv
   HomeWithJointNames.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
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
  LIBRARIES
  CATKIN_DEPENDS roscpp message_runtime
  DEPENDS roscpp icl_core icl_comm icl_hardware driver_svh message_runtime
)

###########
## Build ##
###########

# mesh file generations
# iterate through all the stl files to:
#  1. convert to *.iv files
#  2. generate convex decomposed .stl


foreach(dir description/meshes description/urdf)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

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
  ${icl_hardware_INCLUDE_DIRS}
)

# Additional compiler definitions
add_definitions(
  ${icl_comm_DEFINITIONS}
  ${icl_core_DEFINITIONS}
)

# Declare a cpp executable
add_executable(svh_controller
  src/SVHNode.cpp
  src/DynamicParameterClass.cpp
)

# Simple test programs
add_executable(svh_sin_test
  src/SVHSinTest.cpp
)

# Add cmake target dependencies of the executable/library
# as an example, message headers may need to be generated before nodes
add_dependencies(svh_controller
  icl_comm
  icl_core
  driver_svh
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(svh_controller
  ${catkin_LIBRARIES}
  ${icl_comm_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${driver_svh_LIBRARIES}
  ${icl_hardware_LIBRARIES}
)


add_dependencies(svh_sin_test
  icl_comm
  icl_core
)

## Specify libraries to link a library or executable target against
target_link_libraries(svh_sin_test
  ${catkin_LIBRARIES}
  ${icl_comm_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${icl_hardware_LIBRARIES}
)

## ROSControl
add_library(svh_ros
#   src/SVHRosControlHWInterface.cpp
  src/SVHWrapper.cpp
  src/DynamicParameterClass.cpp
)
add_dependencies(svh_ros
  icl_comm
  icl_core
  driver_svh
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(svh_ros
  ${catkin_LIBRARIES}
  ${icl_comm_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${driver_svh_LIBRARIES}
  ${icl_hardware_LIBRARIES}
)

add_executable(svh_ros_control_node
  src/SVHRosControlHWInterface.cpp
  src/svh_ros_control_node.cpp
)
add_dependencies(svh_ros_control_node
  svh_ros
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(svh_ros_control_node
  svh_ros
  ${catkin_LIBRARIES}
)

## DynamicParametersTest ##

add_executable(svh_dynamic_parameter_test
  src/DynamicParameterTest.cpp
  src/DynamicParameterClass.cpp
)

add_dependencies(svh_dynamic_parameter_test
  icl_comm
  icl_core
  driver_svh
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
install(TARGETS svh_controller svh_sin_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install the headers
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#config files
install(DIRECTORY etc/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/etc)

#udev rules
install(DIRECTORY udev/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/udev)

# Individual files (Quick commands and the urdf.rviz)
install(FILES quick_commands plugin.xml
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

