#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

int occupancyAt(
  const nav_msgs::msg::OccupancyGrid & grid, double world_x, double world_y)
{
  if (grid.info.resolution <= 1e-6 || grid.info.width == 0 || grid.info.height == 0) {
    return -1;
  }
  const int mx = static_cast<int>(
    std::floor((world_x - grid.info.origin.position.x) / grid.info.resolution));
  const int my = static_cast<int>(
    std::floor((world_y - grid.info.origin.position.y) / grid.info.resolution));
  if (mx < 0 || my < 0 ||
    mx >= static_cast<int>(grid.info.width) ||
    my >= static_cast<int>(grid.info.height))
  {
    return -1;
  }
  return grid.data[static_cast<size_t>(my) * grid.info.width + static_cast<size_t>(mx)];
}

int maxOccupancyInDisk(
  const nav_msgs::msg::OccupancyGrid & grid,
  double center_x,
  double center_y,
  double radius)
{
  int worst = 0;
  const double step = std::max(static_cast<double>(grid.info.resolution), 0.05);
  for (double dx = -radius; dx <= radius + 1e-6; dx += step) {
    for (double dy = -radius; dy <= radius + 1e-6; dy += step) {
      if ((dx * dx) + (dy * dy) > (radius * radius) + 1e-6) {
        continue;
      }
      worst = std::max(worst, occupancyAt(grid, center_x + dx, center_y + dy));
    }
  }
  return worst;
}

}  // namespace

class ScoutEscapeInflation : public BT::StatefulActionNode
{
public:
  ScoutEscapeInflation(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    cmd_vel_topic_ = getInput<std::string>("cmd_vel_topic").value_or("/cmd_vel_nav");
    costmap_topic_ = getInput<std::string>("costmap_topic").value_or("/local_costmap/costmap");
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
      callback_group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&ScoutEscapeInflation::costmapCallback, this, std::placeholders::_1),
      options);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("max_distance", 0.45, "Maximum reverse distance in meters"),
      BT::InputPort<double>("speed", 0.08, "Reverse speed in m/s"),
      BT::InputPort<double>("robot_radius", 0.30, "Footprint sample radius in meters"),
      BT::InputPort<int>("enter_cost", 70, "Occupancy that counts as trapped in inflation"),
      BT::InputPort<int>("free_cost", 40, "Occupancy that counts as escaped"),
      BT::InputPort<int>("lethal_cost", 100, "Occupancy treated as a hard obstacle"),
      BT::InputPort<double>("rear_check_distance", 0.40, "Rear corridor length in meters"),
      BT::InputPort<double>("rear_half_width", 0.20, "Rear corridor half-width in meters"),
      BT::InputPort<double>("costmap_timeout", 1.5, "Maximum costmap age in seconds"),
      BT::InputPort<std::string>("cmd_vel_topic", "/cmd_vel_nav", "Velocity command topic"),
      BT::InputPort<std::string>("costmap_topic", "/local_costmap/costmap", "Local costmap")};
  }

  BT::NodeStatus onStart() override
  {
    callback_group_executor_.spin_some();
    max_distance_ = getInput<double>("max_distance").value_or(0.45);
    speed_ = getInput<double>("speed").value_or(0.08);
    robot_radius_ = getInput<double>("robot_radius").value_or(0.30);
    enter_cost_ = getInput<int>("enter_cost").value_or(70);
    free_cost_ = getInput<int>("free_cost").value_or(40);
    lethal_cost_ = getInput<int>("lethal_cost").value_or(100);
    rear_check_distance_ = getInput<double>("rear_check_distance").value_or(0.40);
    rear_half_width_ = getInput<double>("rear_half_width").value_or(0.20);
    costmap_timeout_ = getInput<double>("costmap_timeout").value_or(1.5);
    if (max_distance_ <= 0.05 || speed_ <= 0.01) {
      publishStop();
      return BT::NodeStatus::FAILURE;
    }

    Pose2D pose;
    int robot_cost = 0;
    if (!readPoseAndCosts(&pose, &robot_cost)) {
      publishStop();
      return BT::NodeStatus::FAILURE;
    }
    if (robot_cost < enter_cost_) {
      return BT::NodeStatus::FAILURE;
    }
    if (rearBlocked(pose)) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Inflation escape skipped: robot cost=%d but rear is lethal/unknown",
        robot_cost);
      publishStop();
      return BT::NodeStatus::FAILURE;
    }

    start_pose_ = pose;
    last_tick_ = std::chrono::steady_clock::now();
    traveled_ = 0.0;
    RCLCPP_WARN(
      node_->get_logger(),
      "Robot is inside inflation (cost=%d); reversing at %.2f m/s up to %.2f m",
      robot_cost, speed_, max_distance_);
    publishReverse();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    callback_group_executor_.spin_some();
    Pose2D pose;
    int robot_cost = 0;
    if (!readPoseAndCosts(&pose, &robot_cost)) {
      publishStop();
      return BT::NodeStatus::FAILURE;
    }
    if (rearBlocked(pose)) {
      RCLCPP_WARN(node_->get_logger(), "Inflation escape stopped: rear became lethal");
      publishStop();
      return BT::NodeStatus::FAILURE;
    }

    traveled_ = std::hypot(pose.x - start_pose_.x, pose.y - start_pose_.y);
    if (robot_cost <= free_cost_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "Inflation escape succeeded after %.2f m (cost=%d)",
        traveled_, robot_cost);
      publishStop();
      return BT::NodeStatus::SUCCESS;
    }
    if (traveled_ >= max_distance_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Inflation escape reached %.2f m with remaining cost=%d",
        traveled_, robot_cost);
      publishStop();
      return robot_cost < enter_cost_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    publishReverse();
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    publishStop();
  }

private:
  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    costmap_ = *message;
    have_costmap_ = true;
    costmap_stamp_ = node_->now();
  }

  bool copyCostmap(nav_msgs::msg::OccupancyGrid * grid) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_costmap_) {
      return false;
    }
    if ((node_->now() - costmap_stamp_).seconds() > costmap_timeout_) {
      return false;
    }
    *grid = costmap_;
    return true;
  }

  bool lookupBase(const std::string & frame, Pose2D * pose) const
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        frame, "base_link", tf2::TimePointZero, std::chrono::milliseconds(50));
      pose->x = transform.transform.translation.x;
      pose->y = transform.transform.translation.y;
      const auto & q = transform.transform.rotation;
      pose->yaw = std::atan2(
        2.0 * ((q.w * q.z) + (q.x * q.y)),
        1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z))));
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Inflation escape cannot read %s->base_link: %s", frame.c_str(), error.what());
      return false;
    }
  }

  bool readPoseAndCosts(Pose2D * pose, int * robot_cost)
  {
    nav_msgs::msg::OccupancyGrid grid;
    if (!copyCostmap(&grid) || grid.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Inflation escape has no fresh local costmap on %s", costmap_topic_.c_str());
      return false;
    }
    if (!lookupBase(grid.header.frame_id, pose)) {
      return false;
    }
    *robot_cost = maxOccupancyInDisk(grid, pose->x, pose->y, robot_radius_);
    last_grid_ = std::move(grid);
    last_pose_ = *pose;
    return true;
  }

  bool rearBlocked(const Pose2D & pose) const
  {
    if (last_grid_.data.empty()) {
      return true;
    }
    const double step = std::max(static_cast<double>(last_grid_.info.resolution), 0.05);
    const double cos_yaw = std::cos(pose.yaw);
    const double sin_yaw = std::sin(pose.yaw);
    for (double x = -0.12; x >= -rear_check_distance_ - 1e-6; x -= step) {
      for (double y = -rear_half_width_; y <= rear_half_width_ + 1e-6; y += step) {
        const double wx = pose.x + (x * cos_yaw) - (y * sin_yaw);
        const double wy = pose.y + (x * sin_yaw) + (y * cos_yaw);
        const int cost = occupancyAt(last_grid_, wx, wy);
        if (cost < 0 || cost >= lethal_cost_) {
          return true;
        }
      }
    }
    return false;
  }

  void publishReverse()
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = -std::abs(speed_);
    cmd_pub_->publish(command);
  }

  void publishStop()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  mutable std::mutex mutex_;
  nav_msgs::msg::OccupancyGrid costmap_;
  nav_msgs::msg::OccupancyGrid last_grid_;
  Pose2D start_pose_;
  Pose2D last_pose_;
  rclcpp::Time costmap_stamp_{0, 0, RCL_ROS_TIME};
  std::string cmd_vel_topic_;
  std::string costmap_topic_;
  std::chrono::steady_clock::time_point last_tick_;
  double max_distance_{0.45};
  double speed_{0.08};
  double robot_radius_{0.30};
  double rear_check_distance_{0.40};
  double rear_half_width_{0.20};
  double costmap_timeout_{1.5};
  double traveled_{0.0};
  int enter_cost_{70};
  int free_cost_{40};
  int lethal_cost_{100};
  bool have_costmap_{false};
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ScoutEscapeInflation>("ScoutEscapeInflation");
}
