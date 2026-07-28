cmake_minimum_required(VERSION 3.8)
project(scout_navigation_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch behavior_trees
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/go_to_recorded_pose.py
  scripts/relocalize_and_wait.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
