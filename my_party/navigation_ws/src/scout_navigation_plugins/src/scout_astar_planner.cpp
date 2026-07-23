#include "scout_navigation_plugins/scout_astar_planner.hpp"

#include <stdexcept>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace scout_navigation_plugins
{

void ScoutAstarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  const auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock Nav2 lifecycle node");
  }
  name_ = std::move(name);
  logger_ = node->get_logger();
  costmap_ros_ = std::move(costmap_ros);
  RCLCPP_INFO(logger_, "%s configured", name_.c_str());
}

void ScoutAstarPlanner::cleanup()
{
  costmap_ros_.reset();
}

void ScoutAstarPlanner::activate()
{
  RCLCPP_INFO(logger_, "%s activated", name_.c_str());
}

void ScoutAstarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "%s deactivated", name_.c_str());
}

nav_msgs::msg::Path ScoutAstarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped &,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path empty_path;
  empty_path.header = goal.header;
  RCLCPP_ERROR(logger_, "ScoutAstarPlanner is a scaffold and must not be enabled before A* is implemented");
  return empty_path;
}

}  // namespace scout_navigation_plugins

PLUGINLIB_EXPORT_CLASS(scout_navigation_plugins::ScoutAstarPlanner, nav2_core::GlobalPlanner)
