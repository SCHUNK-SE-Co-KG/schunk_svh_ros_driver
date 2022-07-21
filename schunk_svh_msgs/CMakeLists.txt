cmake_minimum_required(VERSION 3.10)
project(schunk_svh_msgs)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    message_generation
    std_msgs
    std_srvs
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
   std_srvs
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
  CATKIN_DEPENDS
    actionlib_msgs
    message_runtime
    std_msgs
    std_srvs
)

#############
## Install ##
#############

