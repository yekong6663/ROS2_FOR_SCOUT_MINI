cmake_minimum_required(VERSION 3.8)
project(scout_navigation_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/relocalize_and_wait.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
