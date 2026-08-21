#include <algorithm>
#include <array>
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

struct GroundPlane
{
  double a{0.0};
  double b{0.0};
  double c{0.0};
};

bool fitPlane(const std::vector<Point> & points, GroundPlane & plane)
{
  if (points.size() < 30U) {
    return false;
  }

  // Least-squares plane in the sensor frame: z = ax + by + c.  A second
  // inlier-only fit below keeps low returns from curbs and wheels from
  // pulling the fitted road plane upward.
  std::array<std::array<double, 4>, 3> matrix{};
  for (const auto & point : points) {
    const double x = point.x;
    const double y = point.y;
    const double z = point.z;
    matrix[0][0] += x * x;
    matrix[0][1] += x * y;
    matrix[0][2] += x;
    matrix[0][3] += x * z;
    matrix[1][0] += x * y;
    matrix[1][1] += y * y;
    matrix[1][2] += y;
    matrix[1][3] += y * z;
    matrix[2][0] += x;
    matrix[2][1] += y;
    matrix[2][2] += 1.0;
    matrix[2][3] += z;
  }
  for (size_t column = 0; column < 3; ++column) {
    size_t pivot = column;
    for (size_t row = column + 1; row < 3; ++row) {
      if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column])) {
        pivot = row;
      }
    }
    if (std::abs(matrix[pivot][column]) < 1e-8) {
      return false;
    }
    std::swap(matrix[column], matrix[pivot]);
    const double divisor = matrix[column][column];
    for (size_t entry = column; entry < 4; ++entry) {
      matrix[column][entry] /= divisor;
    }
    for (size_t row = 0; row < 3; ++row) {
      if (row == column) {
        continue;
      }
      const double factor = matrix[row][column];
      for (size_t entry = column; entry < 4; ++entry) {
        matrix[row][entry] -= factor * matrix[column][entry];
      }
    }
  }
  plane = GroundPlane{matrix[0][3], matrix[1][3], matrix[2][3]};
  // A fitted near-vertical plane is never a road plane. Reject it instead of
  // accidentally removing a wall or a person when localization is poor.
  return std::abs(plane.a) < 0.45 && std::abs(plane.b) < 0.45;
}

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
    ground_filter_enabled_ = declare_parameter<bool>("ground_filter_enabled", true);
    ground_seed_max_height_ = declare_parameter<double>("ground_seed_max_height", 0.20);
    ground_fit_threshold_ = declare_parameter<double>("ground_fit_threshold", 0.08);
    ground_clearance_ = declare_parameter<double>("ground_clearance", 0.12);
    ground_min_samples_ = declare_parameter<int>("ground_min_samples", 60);

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ObstacleCloudFilter::callback, this, std::placeholders::_1));
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    std::vector<Point> raw_points;
    std::vector<Point> ground_seeds;
    std::vector<Point> candidates;
    std::unordered_map<int64_t, int> occupancy;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z))
        {
          continue;
        }
        const double range = std::hypot(*x, *y);
        if (range < min_range_ || range > max_range_) {
          continue;
        }
        const Point point{*x, *y, *z};
        raw_points.push_back(point);
        // Ground is normally below the sensor. Retain these low points only
        // for plane estimation; they are never published as obstacles.
        if (ground_filter_enabled_ && point.z <= ground_seed_max_height_ &&
          point.z >= -1.50F)
        {
          ground_seeds.push_back(point);
        }
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Obstacle cloud has no usable xyz fields: %s", error.what());
      return;
    }

    GroundPlane ground;
    bool have_ground = false;
    if (ground_filter_enabled_ &&
      ground_seeds.size() >= static_cast<size_t>(std::max(30, ground_min_samples_)))
    {
      GroundPlane first_fit;
      if (fitPlane(ground_seeds, first_fit)) {
        std::vector<Point> inliers;
        inliers.reserve(ground_seeds.size());
        for (const auto & point : ground_seeds) {
          const double predicted = first_fit.a * point.x + first_fit.b * point.y + first_fit.c;
          if (std::abs(point.z - predicted) <= ground_fit_threshold_) {
            inliers.push_back(point);
          }
        }
        have_ground = inliers.size() >= static_cast<size_t>(std::max(30, ground_min_samples_)) &&
          fitPlane(inliers, ground);
      }
    }

    for (const auto & point : raw_points) {
      if (point.z < min_height_ || point.z > max_height_) {
        continue;
      }
      if (have_ground) {
        const double ground_z = ground.a * point.x + ground.b * point.y + ground.c;
        if (point.z <= ground_z + ground_clearance_) {
          continue;
        }
      }
      candidates.push_back(point);
      ++occupancy[cellKey(point.x, point.y)];
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
  bool ground_filter_enabled_;
  double ground_seed_max_height_;
  double ground_fit_threshold_;
  double ground_clearance_;
  int ground_min_samples_;
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
