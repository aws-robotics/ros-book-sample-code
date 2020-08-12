cmake_minimum_required(VERSION 3.5)
project(robot_runtime)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The license note that we use in this repository is not currently supported by ament_copyright
  set(ament_cmake_copyright_FOUND)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
