# This file is licensed under MIT-0 (https://github.com/aws/mit-0)
# which can be found in the 'LICENSE' file in this repository.

cmake_minimum_required(VERSION 3.5)
project(remote_operator)

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
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
