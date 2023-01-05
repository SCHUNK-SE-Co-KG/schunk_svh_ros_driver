cmake_minimum_required(VERSION 3.10)
project(schunk_svh_simulation)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES schunk_svh_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# Install config files
install(DIRECTORY cfg launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
