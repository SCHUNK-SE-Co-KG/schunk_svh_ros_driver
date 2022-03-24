cmake_minimum_required(VERSION 3.5)
project(schunk_svh_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# We need C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(schunk_svh_library REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)


#--------------------------------------------------------------------------------
# Build
#--------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
        src/system_interface.cpp
        src/ROSLogHandler.cpp
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

ament_target_dependencies(${PROJECT_NAME}
        hardware_interface
        pluginlib
        rclcpp
        )

target_link_libraries(${PROJECT_NAME}
        Schunk::svh-library
        )


pluginlib_export_plugin_description_file(hardware_interface schunk_svh_driver.xml)


#--------------------------------------------------------------------------------
# Install
#--------------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY cfg urdf launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  PROGRAMS scripts/example.py
  DESTINATION lib/${PROJECT_NAME}
)

#--------------------------------------------------------------------------------
# Test
#--------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
