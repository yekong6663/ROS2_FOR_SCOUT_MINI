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

add_library(scout_slow_forward_bt_node SHARED
  src/slow_forward_action.cpp)
target_compile_definitions(scout_slow_forward_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(scout_slow_forward_bt_node
  behaviortree_cpp_v3
  geometry_msgs
  rclcpp
  sensor_msgs)

add_library(scout_escape_inflation_bt_node SHARED
  src/escape_inflation_action.cpp)
target_compile_definitions(scout_escape_inflation_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(scout_escape_inflation_bt_node
  behaviortree_cpp_v3
  geometry_msgs
  nav_msgs
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

add_executable(obstacle_cloud_filter src/obstacle_cloud_filter.cpp)
ament_target_dependencies(obstacle_cloud_filter rclcpp sensor_msgs)

install(DIRECTORY config launch behavior_trees
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/dock_to_recorded_point3.py
  scripts/safe_startup_forward.py
  scripts/dock_to_indoor_grasp_point.py
  scripts/dock_to_indoor_place_point.py
  scripts/go_to_recorded_pose.py
  scripts/ensure_map_server_active.py
  scripts/run_outdoor2_recorded_route.py
  scripts/run_outdoor2_recorded_route.sh
  scripts/run_outdoor03_recorded_route.py
  scripts/run_outdoor03_recorded_route.sh
  scripts/run_outdoor_recorded_route.py
  scripts/run_indoor_recorded_route.sh
  scripts/run_indoor03_recorded_route.sh
  scripts/run_indoor03_recorded_route.py
  scripts/relocalize_and_wait.py
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  scout_near_goal_condition_bt_node
  scout_slow_forward_bt_node
  scout_escape_inflation_bt_node
  scout_dynamic_obstacle_buffer_layer
  LIBRARY DESTINATION lib)

install(TARGETS obstacle_cloud_filter
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(FILES dynamic_obstacle_buffer_plugins.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
