cmake_minimum_required(VERSION 3.8)
project(scout_navigation_bringup)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(scout_near_goal_condition_bt_node SHARED
  src/near_goal_condition.cpp)
target_compile_definitions(scout_near_goal_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(scout_near_goal_condition_bt_node
  behaviortree_cpp_v3
  geometry_msgs
  rclcpp
  tf2_ros)

add_library(scout_dynamic_obstacle_buffer_layer SHARED
  src/dynamic_obstacle_buffer_layer.cpp)
ament_target_dependencies(scout_dynamic_obstacle_buffer_layer
  geometry_msgs
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros)
pluginlib_export_plugin_description_file(nav2_costmap_2d dynamic_obstacle_buffer_plugins.xml)

install(DIRECTORY config launch behavior_trees
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/go_to_recorded_pose.py
  scripts/run_outdoor_recorded_route.py
  scripts/relocalize_and_wait.py
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  scout_near_goal_condition_bt_node
  scout_dynamic_obstacle_buffer_layer
  LIBRARY DESTINATION lib)

install(FILES dynamic_obstacle_buffer_plugins.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
