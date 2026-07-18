cmake_minimum_required(VERSION 3.8)
project(pointcloud_map_projection)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)

add_executable(pointcloud_map_projection_node src/pointcloud_map_projection_node.cpp)
target_include_directories(pointcloud_map_projection_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pointcloud_map_projection_node ${PCL_LIBRARIES})
ament_target_dependencies(pointcloud_map_projection_node rclcpp nav_msgs)

install(TARGETS pointcloud_map_projection_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
