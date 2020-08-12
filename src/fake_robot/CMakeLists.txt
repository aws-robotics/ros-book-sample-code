# This file is licensed under MIT-0 (https://github.com/aws/mit-0)
# which can be found in the 'LICENSE' file in this repository.

cmake_minimum_required(VERSION 3.5)
project(fake_robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(libstatistics_collector REQUIRED)
find_package(statistics_msgs REQUIRED)

include_directories(include)

add_library(fake_laser_component SHARED src/fake_laser.cpp)
target_compile_definitions(fake_laser_component PRIVATE "FAKE_ROBOT_BUILDING_DLL")
ament_target_dependencies(fake_laser_component "rclcpp" "rclcpp_components" "sensor_msgs")
rclcpp_components_register_node(fake_laser_component PLUGIN "fake_robot::FakeLaserNode" EXECUTABLE fake_laser_node)

add_library(laser_listener_component SHARED src/laser_listener.cpp)
target_compile_definitions(laser_listener_component PRIVATE "FAKE_ROBOT_BUILDING_DLL")
ament_target_dependencies(laser_listener_component "rclcpp" "rclcpp_components" "sensor_msgs")
rclcpp_components_register_node(laser_listener_component PLUGIN "fake_robot::LaserListenerNode" EXECUTABLE laser_listener_node)

install(TARGETS
  fake_laser_component
  laser_listener_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The license note that we use in this repository is not currently supported by ament_copyright
  set(ament_cmake_copyright_FOUND)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
