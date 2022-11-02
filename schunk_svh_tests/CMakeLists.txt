cmake_minimum_required(VERSION 3.10)
project(schunk_svh_tests)

find_package(ament_cmake REQUIRED)


if(BUILD_TESTING)
  find_package(schunk_svh_driver REQUIRED)
  find_package(controller_manager_msgs REQUIRED)
  find_package(launch_testing_ament_cmake)
  add_launch_test(integration_tests/test_startup.py)
endif()

ament_package()
