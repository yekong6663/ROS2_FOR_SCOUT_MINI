cmake_minimum_required(VERSION 3.8)
project(scout_navigation_plugins)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(scout_astar_planner SHARED src/scout_astar_planner.cpp)
target_include_directories(scout_astar_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(scout_astar_planner
  geometry_msgs nav2_core nav2_costmap_2d nav_msgs pluginlib rclcpp rclcpp_lifecycle tf2_ros)

pluginlib_export_plugin_description_file(nav2_core scout_navigation_plugins.xml)

install(TARGETS scout_astar_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include DESTINATION include)
install(FILES scout_navigation_plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
