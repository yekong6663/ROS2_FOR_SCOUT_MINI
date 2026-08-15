#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{

struct Point
{
  float x;
  float y;
  float z;
};

class ObstacleCloudFilter : public rclcpp::Node
{
public:
  ObstacleCloudFilter()
  : Node("obstacle_cloud_filter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/fastlio2/body_cloud");
    output_topic_ = declare_parameter<std::string>("output_topic", "/nav/filtered_obstacle_cloud");
    min_height_ = declare_parameter<double>("min_obstacle_height", -0.05);
    max_height_ = declare_parameter<double>("max_obstacle_height", 1.35);
    min_range_ = declare_parameter<double>("min_range", 0.35);
    max_range_ = declare_parameter<double>("max_range", 5.0);
    cell_size_ = declare_parameter<double>("cell_size", 0.25);
    min_points_per_cell_ = declare_parameter<int>("min_points_per_cell", 8);

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ObstacleCloudFilter::callback, this, std::placeholders::_1));
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    std::vector<Point> candidates;
    std::unordered_map<int64_t, int> occupancy;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z) ||
          *z < min_height_ || *z > max_height_)
        {
          continue;
        }
        const double range = std::hypot(*x, *y);
        if (range < min_range_ || range > max_range_) {
          continue;
        }
        candidates.push_back(Point{*x, *y, *z});
        ++occupancy[cellKey(*x, *y)];
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Obstacle cloud has no usable xyz fields: %s", error.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 filtered;
    filtered.header = cloud->header;
    filtered.height = 1;
    filtered.is_bigendian = false;
    filtered.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(filtered);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    const size_t accepted = std::count_if(candidates.begin(), candidates.end(),
        [this, &occupancy](const Point & point) {
          return occupancy.at(cellKey(point.x, point.y)) >= min_points_per_cell_;
        });
    modifier.resize(accepted);
    sensor_msgs::PointCloud2Iterator<float> out_x(filtered, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(filtered, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(filtered, "z");
    for (const auto & point : candidates) {
      if (occupancy.at(cellKey(point.x, point.y)) < min_points_per_cell_) {
        continue;
      }
      *out_x = point.x;
      *out_y = point.y;
      *out_z = point.z;
      ++out_x;
      ++out_y;
      ++out_z;
    }
    publisher_->publish(filtered);
  }

  int64_t cellKey(float x, float y) const
  {
    const int32_t gx = static_cast<int32_t>(std::floor(x / cell_size_));
    const int32_t gy = static_cast<int32_t>(std::floor(y / cell_size_));
    return (static_cast<int64_t>(gx) << 32) ^ static_cast<uint32_t>(gy);
  }

  std::string input_topic_;
  std::string output_topic_;
  double min_height_;
  double max_height_;
  double min_range_;
  double max_range_;
  double cell_size_;
  int min_points_per_cell_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleCloudFilter>());
  rclcpp::shutdown();
  return 0;
}
