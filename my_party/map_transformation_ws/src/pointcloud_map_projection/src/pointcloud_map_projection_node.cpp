#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"

class PointcloudMapProjectionNode : public rclcpp::Node
{
public:
  PointcloudMapProjectionNode() : Node("pointcloud_map_projection")
  {
    pcd_file_ = declare_parameter<std::string>("pcd_file", "");
    output_topic_ = declare_parameter<std::string>("output_topic", "/map");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    resolution_ = declare_parameter<double>("resolution", 0.05);
    z_min_ = declare_parameter<double>("z_min", 0.15);
    z_max_ = declare_parameter<double>("z_max", 1.20);
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.05);
    enable_radius_filter_ = declare_parameter<bool>("enable_radius_filter", true);
    radius_search_ = declare_parameter<double>("radius_search", 0.15);
    min_neighbors_ = declare_parameter<int>("min_neighbors", 3);
    map_padding_ = declare_parameter<double>("map_padding", 0.50);
    unobserved_value_ = declare_parameter<int>("unobserved_value", -1);

    if (!validateParameters()) {
      return;
    }

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      output_topic_, rclcpp::QoS(1).transient_local());
    ready_ = buildMap();
    if (ready_) {
      publishMap();
    }
  }

  bool ready() const
  {
    return ready_;
  }

private:
  bool validateParameters()
  {
    if (pcd_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'pcd_file' must point to a saved GlobalMap.pcd");
      return false;
    }
    if (resolution_ <= 0.0 || z_min_ >= z_max_ || voxel_leaf_size_ <= 0.0 ||
      radius_search_ <= 0.0 || min_neighbors_ <= 0 || map_padding_ < 0.0 ||
      unobserved_value_ < -1 || unobserved_value_ > 100)
    {
      RCLCPP_ERROR(get_logger(), "Invalid map projection parameters");
      return false;
    }
    return true;
  }

  bool buildMap()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(pcd_file_, *source) < 0) {
      RCLCPP_ERROR(get_logger(), "Could not load PCD file: %s", pcd_file_.c_str());
      return false;
    }

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(source);
    voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter.filter(*downsampled);

    pcl::PassThrough<pcl::PointXYZ> z_filter;
    z_filter.setInputCloud(downsampled);
    z_filter.setFilterFieldName("z");
    z_filter.setFilterLimits(z_min_, z_max_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    z_filter.filter(*filtered);

    if (enable_radius_filter_) {
      pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
      radius_filter.setInputCloud(filtered);
      radius_filter.setRadiusSearch(radius_search_);
      radius_filter.setMinNeighborsInRadius(min_neighbors_);
      pcl::PointCloud<pcl::PointXYZ>::Ptr denoised(new pcl::PointCloud<pcl::PointXYZ>);
      radius_filter.filter(*denoised);
      filtered = denoised;
    }

    if (filtered->empty()) {
      RCLCPP_ERROR(get_logger(), "No points remain after filtering; check z_min, z_max and filters");
      return false;
    }

    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::lowest();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::lowest();
    for (const auto & point : filtered->points) {
      x_min = std::min(x_min, static_cast<double>(point.x));
      x_max = std::max(x_max, static_cast<double>(point.x));
      y_min = std::min(y_min, static_cast<double>(point.y));
      y_max = std::max(y_max, static_cast<double>(point.y));
    }

    grid_.info.resolution = resolution_;
    grid_.info.origin.position.x = x_min - map_padding_;
    grid_.info.origin.position.y = y_min - map_padding_;
    grid_.info.origin.orientation.w = 1.0;
    grid_.info.width = static_cast<uint32_t>(std::ceil(
      (x_max - x_min + 2.0 * map_padding_) / resolution_));
    grid_.info.height = static_cast<uint32_t>(std::ceil(
      (y_max - y_min + 2.0 * map_padding_) / resolution_));
    grid_.data.assign(
      static_cast<size_t>(grid_.info.width) * grid_.info.height,
      static_cast<int8_t>(unobserved_value_));

    for (const auto & point : filtered->points) {
      const int mx = static_cast<int>(std::floor(
        (point.x - grid_.info.origin.position.x) / resolution_));
      const int my = static_cast<int>(std::floor(
        (point.y - grid_.info.origin.position.y) / resolution_));
      if (mx >= 0 && mx < static_cast<int>(grid_.info.width) &&
        my >= 0 && my < static_cast<int>(grid_.info.height))
      {
        grid_.data[static_cast<size_t>(my) * grid_.info.width + mx] = 100;
      }
    }

    RCLCPP_INFO(
      get_logger(), "Generated %u x %u map from %zu / %zu PCD points",
      grid_.info.width, grid_.info.height, filtered->size(), source->size());
    return true;
  }

  void publishMap()
  {
    grid_.header.stamp = now();
    grid_.header.frame_id = frame_id_;
    map_pub_->publish(grid_);
  }

  std::string pcd_file_;
  std::string output_topic_;
  std::string frame_id_;
  double resolution_{};
  double z_min_{};
  double z_max_{};
  double voxel_leaf_size_{};
  bool enable_radius_filter_{};
  double radius_search_{};
  int min_neighbors_{};
  double map_padding_{};
  int unobserved_value_{};
  bool ready_{false};

  nav_msgs::msg::OccupancyGrid grid_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<PointcloudMapProjectionNode>();
  if (!node->ready()) {
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
