cmake_minimum_required(VERSION 3.8)
project(slam_bot_nav)

find_package(ament_cmake REQUIRED)

# Config-only package: install the parameter YAMLs where launch files find them.
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
