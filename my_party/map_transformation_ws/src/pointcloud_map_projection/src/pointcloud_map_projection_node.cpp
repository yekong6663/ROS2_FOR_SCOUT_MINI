#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PointcloudMapProjectionNode : public rclcpp::Node
{
public:
  PointcloudMapProjectionNode() : Node("pointcloud_map_projection")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/fastlio2/world_cloud");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/fastlio2/lio_odom");
    output_topic_ = declare_parameter<std::string>("output_topic", "/projected_map");
    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    resolution_ = declare_parameter<double>("resolution", 0.10);
    width_ = declare_parameter<int>("width", 800);
    height_ = declare_parameter<int>("height", 800);
    origin_x_ = declare_parameter<double>("origin_x", -40.0);
    origin_y_ = declare_parameter<double>("origin_y", -40.0);
    obstacle_min_height_ = declare_parameter<double>("obstacle_min_height", 0.15);
    obstacle_max_height_ = declare_parameter<double>("obstacle_max_height", 1.20);
    min_hits_ = declare_parameter<int>("min_hits", 2);
    mark_free_space_ = declare_parameter<bool>("mark_free_space", true);

    if (resolution_ <= 0.0 || width_ <= 0 || height_ <= 0 || min_hits_ <= 0) {
      throw std::runtime_error("resolution, width, height and min_hits must be positive");
    }

    grid_.info.resolution = resolution_;
    grid_.info.width = static_cast<uint32_t>(width_);
    grid_.info.height = static_cast<uint32_t>(height_);
    grid_.info.origin.position.x = origin_x_;
    grid_.info.origin.position.y = origin_y_;
    grid_.info.origin.orientation.w = 1.0;
    grid_.data.assign(static_cast<size_t>(width_ * height_), -1);
    hits_.assign(grid_.data.size(), 0);

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      output_topic_, rclcpp::QoS(1).transient_local());
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      std::bind(&PointcloudMapProjectionNode::odomCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PointcloudMapProjectionNode::cloudCallback, this, std::placeholders::_1));
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    sensor_x_ = msg->pose.pose.position.x;
    sensor_y_ = msg->pose.pose.position.y;
    have_odom_ = true;
  }

  bool toCell(double x, double y, int & mx, int & my) const
  {
    mx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    my = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    return mx >= 0 && mx < width_ && my >= 0 && my < height_;
  }

  size_t index(int mx, int my) const
  {
    return static_cast<size_t>(my * width_ + mx);
  }

  void markFreeRay(int x0, int y0, int x1, int y1)
  {
    int dx = std::abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;

    while (x0 != x1 || y0 != y1) {
      const auto cell = index(x0, y0);
      if (grid_.data[cell] != 100) {
        grid_.data[cell] = 0;
      }
      const int twice_error = 2 * error;
      if (twice_error >= dy) {
        error += dy;
        x0 += sx;
      }
      if (twice_error <= dx) {
        error += dx;
        y0 += sy;
      }
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    int sensor_mx = 0;
    int sensor_my = 0;
    const bool can_mark_free = mark_free_space_ && have_odom_ &&
      toCell(sensor_x_, sensor_y_, sensor_mx, sensor_my);

    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      int mx = 0;
      int my = 0;
      if (!toCell(point.x, point.y, mx, my)) {
        continue;
      }

      if (can_mark_free) {
        markFreeRay(sensor_mx, sensor_my, mx, my);
      }

      const auto cell = index(mx, my);
      if (point.z < obstacle_min_height_ || point.z > obstacle_max_height_) {
        if (grid_.data[cell] == -1) {
          grid_.data[cell] = 0;
        }
        continue;
      }

      hits_[cell] = std::min<uint16_t>(hits_[cell] + 1, UINT16_MAX);
      if (hits_[cell] >= min_hits_) {
        grid_.data[cell] = 100;
      }
    }

    grid_.header.stamp = msg->header.stamp;
    grid_.header.frame_id = frame_id_.empty() ? msg->header.frame_id : frame_id_;
    map_pub_->publish(grid_);
  }

  std::string input_topic_;
  std::string odom_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double resolution_{};
  int width_{};
  int height_{};
  double origin_x_{};
  double origin_y_{};
  double obstacle_min_height_{};
  double obstacle_max_height_{};
  int min_hits_{};
  bool mark_free_space_{};
  bool have_odom_{false};
  double sensor_x_{};
  double sensor_y_{};

  nav_msgs::msg::OccupancyGrid grid_;
  std::vector<uint16_t> hits_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudMapProjectionNode>());
  rclcpp::shutdown();
  return 0;
}
