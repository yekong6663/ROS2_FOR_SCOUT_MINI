#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class ScoutSlowForward : public BT::StatefulActionNode
{
public:
  ScoutSlowForward(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    const auto topic = getInput<std::string>("cmd_vel_topic");
    cmd_vel_topic_ = topic ? topic.value() : "/cmd_vel_nav";
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    safety_enabled_ = getInput<bool>("front_safety_enabled").value_or(true);
    front_stop_distance_ = getInput<double>("front_stop_distance").value_or(0.45);
    front_half_width_ = getInput<double>("front_half_width").value_or(0.18);
    front_min_points_ = getInput<int>("front_min_points").value_or(4);
    cloud_timeout_ = getInput<double>("cloud_timeout").value_or(0.5);
    const auto cloud_topic =
      getInput<std::string>("cloud_topic").value_or("/fastlio2/body_cloud");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
      callback_group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group_;
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&ScoutSlowForward::cloudCallback, this, std::placeholders::_1),
      subscription_options);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("speed", 0.08, "Forward speed in m/s"),
      BT::InputPort<double>("duration", 1.5, "Forward command duration in seconds"),
      BT::InputPort<std::string>("cmd_vel_topic", "/cmd_vel_nav", "Velocity command topic"),
      BT::InputPort<bool>("front_safety_enabled", true, "Pause for a real object ahead"),
      BT::InputPort<std::string>("cloud_topic", "/fastlio2/body_cloud", "Body-frame cloud"),
      BT::InputPort<double>("front_stop_distance", 0.45, "Front stop distance in meters"),
      BT::InputPort<double>("front_half_width", 0.18, "Half width of front stop corridor"),
      BT::InputPort<int>("front_min_points", 4, "Point count required to stop"),
      BT::InputPort<double>("cloud_timeout", 0.5, "Maximum point-cloud age in seconds")};
  }

  BT::NodeStatus onStart() override
  {
    callback_group_executor_.spin_some();
    const auto speed = getInput<double>("speed");
    const auto duration = getInput<double>("duration");
    if (!speed || !duration || speed.value() <= 0.0 || duration.value() <= 0.0) {
      RCLCPP_ERROR(node_->get_logger(), "ScoutSlowForward received invalid speed or duration");
      publishStop();
      return BT::NodeStatus::FAILURE;
    }

    speed_ = speed.value();
    duration_ = std::chrono::duration<double>(duration.value());
    moving_time_ = std::chrono::duration<double>(0.0);
    last_tick_ = std::chrono::steady_clock::now();
    RCLCPP_WARN(
      node_->get_logger(),
      "Normal navigation failed; slow-forward recovery started: %.2f m/s for %.1f s",
      speed_, duration.value());
    if (frontBlocked()) {
      publishStop();
    } else {
      publishForward();
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    callback_group_executor_.spin_some();
    const auto now = std::chrono::steady_clock::now();
    const auto tick_duration = now - last_tick_;
    last_tick_ = now;

    if (frontBlocked()) {
      publishStop();
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Slow-forward motion paused: person or object detected in front");
      return BT::NodeStatus::RUNNING;
    }

    moving_time_ += tick_duration;
    if (moving_time_ >= duration_) {
      // Keep the last slow-forward command active while Nav2 computes and
      // starts the replacement path.  The controller's first command takes
      // over the same topic, avoiding an intentional zero-speed transition.
      RCLCPP_INFO(
        node_->get_logger(),
        "Slow-forward recovery finished; handing motion directly to Nav2 replanning");
      return BT::NodeStatus::SUCCESS;
    }
    publishForward();
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    publishStop();
  }

private:
  static int64_t steadyNowNanoseconds()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    last_cloud_ns_.store(steadyNowNanoseconds());
    if (!safety_enabled_) {
      front_blocked_.store(false);
      return;
    }

    int count = 0;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (
          *x > 0.20 && *x < front_stop_distance_ &&
          std::abs(*y) < front_half_width_ && *z > -0.15 && *z < 1.70)
        {
          ++count;
          if (count >= front_min_points_) {
            front_blocked_.store(true);
            return;
          }
        }
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Slow-forward safety cloud parse failed: %s", error.what());
      front_blocked_.store(true);
      return;
    }
    front_blocked_.store(false);
  }

  bool frontBlocked() const
  {
    if (!safety_enabled_) {
      return false;
    }
    const int64_t age_ns = steadyNowNanoseconds() - last_cloud_ns_.load();
    const int64_t timeout_ns = static_cast<int64_t>(cloud_timeout_ * 1e9);
    return last_cloud_ns_.load() == 0 || age_ns > timeout_ns || front_blocked_.load();
  }

  void publishForward()
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = speed_;
    cmd_pub_->publish(command);
  }

  void publishStop()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::string cmd_vel_topic_;
  std::chrono::steady_clock::time_point last_tick_;
  std::chrono::duration<double> duration_{0.0};
  std::chrono::duration<double> moving_time_{0.0};
  std::atomic<bool> front_blocked_{true};
  std::atomic<int64_t> last_cloud_ns_{0};
  bool safety_enabled_{true};
  double front_stop_distance_{0.45};
  double front_half_width_{0.18};
  int front_min_points_{4};
  double cloud_timeout_{0.5};
  double speed_{0.0};
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ScoutSlowForward>("ScoutSlowForward");
}
