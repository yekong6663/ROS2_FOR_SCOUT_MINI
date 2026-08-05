#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace scout_navigation_bringup
{

class DynamicObstacleBufferLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    auto node = node_.lock();
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("topic", rclcpp::ParameterValue("/fastlio2/body_cloud"));
    declareParameter("map_topic", rclcpp::ParameterValue("/map"));
    declareParameter("inflation_radius", rclcpp::ParameterValue(0.55));
    declareParameter("inscribed_radius", rclcpp::ParameterValue(0.34));
    declareParameter("cost_scaling_factor", rclcpp::ParameterValue(4.0));
    declareParameter("min_obstacle_height", rclcpp::ParameterValue(-0.15));
    declareParameter("max_obstacle_height", rclcpp::ParameterValue(1.70));
    declareParameter("obstacle_min_range", rclcpp::ParameterValue(0.35));
    declareParameter("obstacle_max_range", rclcpp::ParameterValue(6.0));
    declareParameter("observation_timeout", rclcpp::ParameterValue(0.30));
    declareParameter("cluster_resolution", rclcpp::ParameterValue(0.20));
    declareParameter("min_points_per_cluster", rclcpp::ParameterValue(3));

    node->get_parameter(name_ + ".enabled", enabled_);
    node->get_parameter(name_ + ".topic", topic_);
    node->get_parameter(name_ + ".map_topic", map_topic_);
    node->get_parameter(name_ + ".inflation_radius", inflation_radius_);
    node->get_parameter(name_ + ".inscribed_radius", inscribed_radius_);
    node->get_parameter(name_ + ".cost_scaling_factor", cost_scaling_factor_);
    node->get_parameter(name_ + ".min_obstacle_height", min_obstacle_height_);
    node->get_parameter(name_ + ".max_obstacle_height", max_obstacle_height_);
    node->get_parameter(name_ + ".obstacle_min_range", min_range_);
    node->get_parameter(name_ + ".obstacle_max_range", max_range_);
    node->get_parameter(name_ + ".observation_timeout", observation_timeout_);
    node->get_parameter(name_ + ".cluster_resolution", cluster_resolution_);
    node->get_parameter(name_ + ".min_points_per_cluster", min_points_per_cluster_);
    resetBounds(current_min_x_, current_min_y_, current_max_x_, current_max_y_);
    resetBounds(last_min_x_, last_min_y_, last_max_x_, last_max_y_);
    last_observation_time_ = clock_->now() - rclcpp::Duration::from_seconds(observation_timeout_ + 1.0);

    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_, rclcpp::SensorDataQoS(),
      std::bind(&DynamicObstacleBufferLayer::cloudCallback, this, std::placeholders::_1),
      options);
    map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&DynamicObstacleBufferLayer::mapCallback, this, std::placeholders::_1),
      options);
    current_ = true;
  }

  void updateBounds(
    double, double, double, double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    includeBounds(last_min_x_, last_min_y_, last_max_x_, last_max_y_, min_x, min_y, max_x, max_y);
    if (isFresh()) {
      includeBounds(current_min_x_, current_min_y_, current_max_x_, current_max_y_, min_x, min_y, max_x, max_y);
    }
  }

  void updateCosts(nav2_costmap_2d::Costmap2D & master, int, int, int, int) override
  {
    std::vector<Point> points;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (isFresh()) {
        points = points_;
        last_min_x_ = current_min_x_;
        last_min_y_ = current_min_y_;
        last_max_x_ = current_max_x_;
        last_max_y_ = current_max_y_;
      } else {
        last_min_x_ = current_min_x_;
        last_min_y_ = current_min_y_;
        last_max_x_ = current_max_x_;
        last_max_y_ = current_max_y_;
      }
    }

    const double resolution = master.getResolution();
    const int cell_radius = static_cast<int>(std::ceil(inflation_radius_ / resolution));
    for (const auto & point : points) {
      unsigned int center_x;
      unsigned int center_y;
      if (!master.worldToMap(point.x, point.y, center_x, center_y)) {
        continue;
      }
      for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
        for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
          const int mx = static_cast<int>(center_x) + dx;
          const int my = static_cast<int>(center_y) + dy;
          if (mx < 0 || my < 0 || mx >= static_cast<int>(master.getSizeInCellsX()) ||
            my >= static_cast<int>(master.getSizeInCellsY()))
          {
            continue;
          }
          const double distance = std::hypot(dx * resolution, dy * resolution);
          if (distance > inflation_radius_) {
            continue;
          }
          const auto cost = dynamicCost(distance);
          if (cost > master.getCost(static_cast<unsigned int>(mx), static_cast<unsigned int>(my))) {
            master.setCost(static_cast<unsigned int>(mx), static_cast<unsigned int>(my), cost);
          }
        }
      }
    }
  }

  void reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    points_.clear();
    resetBounds(current_min_x_, current_min_y_, current_max_x_, current_max_y_);
  }

  bool isClearable() override {return true;}

private:
  struct Point {double x; double y;};
  struct Cluster {double x_sum{0.0}; double y_sum{0.0}; int count{0};};

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    static_map_ = map;
  }

  bool isSavedMapObstacle(double x, double y) const
  {
    nav_msgs::msg::OccupancyGrid::SharedPtr map;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map = static_map_;
    }
    if (!map || map->info.resolution <= 0.0) {
      return false;
    }
    geometry_msgs::msg::PointStamped source;
    geometry_msgs::msg::PointStamped in_map;
    source.point.x = x;
    source.point.y = y;
    try {
      const auto transform = tf_->lookupTransform(
        map->header.frame_id, layered_costmap_->getGlobalFrameID(), tf2::TimePointZero);
      tf2::doTransform(source, in_map, transform);
    } catch (const tf2::TransformException &) {
      return false;
    }
    const auto & origin = map->info.origin;
    const double yaw = tf2::getYaw(origin.orientation);
    const double dx = in_map.point.x - origin.position.x;
    const double dy = in_map.point.y - origin.position.y;
    const int mx = static_cast<int>(std::floor((std::cos(yaw) * dx + std::sin(yaw) * dy) / map->info.resolution));
    const int my = static_cast<int>(std::floor((-std::sin(yaw) * dx + std::cos(yaw) * dy) / map->info.resolution));
    if (mx < 0 || my < 0 || mx >= static_cast<int>(map->info.width) || my >= static_cast<int>(map->info.height)) {
      return true;
    }
    const auto occupancy = map->data[static_cast<size_t>(my) * map->info.width + mx];
    return occupancy != 0;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(
        layered_costmap_->getGlobalFrameID(), cloud->header.frame_id,
        rclcpp::Time(cloud->header.stamp), rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Dynamic obstacle buffer skipped cloud: %s", ex.what());
      return;
    }

    std::unordered_map<int64_t, Cluster> clusters;
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z) ||
          *z < min_obstacle_height_ || *z > max_obstacle_height_)
        {
          continue;
        }
        const double range = std::hypot(*x, *y);
        if (range < min_range_ || range > max_range_) {
          continue;
        }
        geometry_msgs::msg::PointStamped input;
        geometry_msgs::msg::PointStamped output;
        input.point.x = *x;
        input.point.y = *y;
        input.point.z = *z;
        tf2::doTransform(input, output, transform);
        if (isSavedMapObstacle(output.point.x, output.point.y)) {
          continue;
        }
        const int gx = static_cast<int>(std::floor(output.point.x / cluster_resolution_));
        const int gy = static_cast<int>(std::floor(output.point.y / cluster_resolution_));
        const int64_t key = (static_cast<int64_t>(gx) << 32) ^ static_cast<uint32_t>(gy);
        auto & cluster = clusters[key];
        cluster.x_sum += output.point.x;
        cluster.y_sum += output.point.y;
        ++cluster.count;
      }
    } catch (const std::runtime_error & ex) {
      RCLCPP_WARN(logger_, "Dynamic obstacle buffer received an invalid PointCloud2: %s", ex.what());
      return;
    }

    std::vector<Point> received;
    received.reserve(clusters.size());
    for (const auto & entry : clusters) {
      const auto & cluster = entry.second;
      if (cluster.count < min_points_per_cluster_) {
        continue;
      }
      const Point point{cluster.x_sum / cluster.count, cluster.y_sum / cluster.count};
      received.push_back(point);
      min_x = std::min(min_x, point.x - inflation_radius_);
      min_y = std::min(min_y, point.y - inflation_radius_);
      max_x = std::max(max_x, point.x + inflation_radius_);
      max_y = std::max(max_y, point.y + inflation_radius_);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    points_ = std::move(received);
    if (points_.empty()) {
      resetBounds(current_min_x_, current_min_y_, current_max_x_, current_max_y_);
    } else {
      current_min_x_ = min_x;
      current_min_y_ = min_y;
      current_max_x_ = max_x;
      current_max_y_ = max_y;
    }
    last_observation_time_ = clock_->now();
  }

  unsigned char dynamicCost(double distance) const
  {
    if (distance <= inscribed_radius_) {
      return nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    }
    const double scaled = std::exp(-cost_scaling_factor_ * (distance - inscribed_radius_));
    return static_cast<unsigned char>(
      std::max(1.0, scaled * (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)));
  }

  bool isFresh() const
  {
    return (clock_->now() - last_observation_time_).seconds() <= observation_timeout_;
  }

  static void resetBounds(double & min_x, double & min_y, double & max_x, double & max_y)
  {
    min_x = std::numeric_limits<double>::infinity();
    min_y = std::numeric_limits<double>::infinity();
    max_x = -std::numeric_limits<double>::infinity();
    max_y = -std::numeric_limits<double>::infinity();
  }

  static void includeBounds(
    double from_min_x, double from_min_y, double from_max_x, double from_max_y,
    double * min_x, double * min_y, double * max_x, double * max_y)
  {
    if (!std::isfinite(from_min_x)) {
      return;
    }
    *min_x = std::min(*min_x, from_min_x);
    *min_y = std::min(*min_y, from_min_y);
    *max_x = std::max(*max_x, from_max_x);
    *max_y = std::max(*max_y, from_max_y);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  std::mutex mutex_;
  mutable std::mutex map_mutex_;
  std::vector<Point> points_;
  nav_msgs::msg::OccupancyGrid::SharedPtr static_map_;
  std::string topic_;
  std::string map_topic_;
  double inflation_radius_{0.55};
  double inscribed_radius_{0.34};
  double cost_scaling_factor_{4.0};
  double min_obstacle_height_{-0.15};
  double max_obstacle_height_{1.70};
  double min_range_{0.35};
  double max_range_{6.0};
  double observation_timeout_{0.30};
  double cluster_resolution_{0.20};
  int min_points_per_cluster_{3};
  rclcpp::Time last_observation_time_{0, 0, RCL_ROS_TIME};
  double current_min_x_;
  double current_min_y_;
  double current_max_x_;
  double current_max_y_;
  double last_min_x_;
  double last_min_y_;
  double last_max_x_;
  double last_max_y_;
};

}  // namespace scout_navigation_bringup

PLUGINLIB_EXPORT_CLASS(
  scout_navigation_bringup::DynamicObstacleBufferLayer, nav2_costmap_2d::Layer)
