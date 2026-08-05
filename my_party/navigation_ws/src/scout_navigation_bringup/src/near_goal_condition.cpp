#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class ScoutNearGoal : public BT::ConditionNode
{
public:
  ScoutNearGoal(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<double>("distance", 1.0, "Maximum distance from base to goal")};
  }

  BT::NodeStatus tick() override
  {
    const auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
    const auto threshold = getInput<double>("distance");
    if (!goal || !threshold || goal->header.frame_id.empty() || *threshold <= 0.0) {
      RCLCPP_WARN(node_->get_logger(), "ScoutNearGoal has an invalid goal or distance");
      return BT::NodeStatus::FAILURE;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        goal->header.frame_id, "base_link", tf2::TimePointZero);
      const double dx = goal->pose.position.x - transform.transform.translation.x;
      const double dy = goal->pose.position.y - transform.transform.translation.y;
      return std::hypot(dx, dy) <= *threshold ?
             BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "ScoutNearGoal cannot read goal transform: %s", ex.what());
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ScoutNearGoal>("ScoutNearGoal");
}
