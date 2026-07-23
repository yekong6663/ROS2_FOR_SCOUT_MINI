#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl/common/transforms.h"
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
    poses_file_ = declare_parameter<std::string>("poses_file", "");
    patches_dir_ = declare_parameter<std::string>("patches_dir", "");
    remove_self_points_ = declare_parameter<bool>("remove_self_points", false);
    self_min_x_ = declare_parameter<double>("self_filter.min_x", -0.65);
    self_max_x_ = declare_parameter<double>("self_filter.max_x", 0.65);
    self_min_y_ = declare_parameter<double>("self_filter.min_y", -0.55);
    self_max_y_ = declare_parameter<double>("self_filter.max_y", 0.55);
    self_min_z_ = declare_parameter<double>("self_filter.min_z", -1.00);
    self_max_z_ = declare_parameter<double>("self_filter.max_z", 0.35);
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
    free_space_radius_ = declare_parameter<double>("free_space_radius", 0.80);
    trajectory_clear_radius_ = declare_parameter<double>("trajectory_clear_radius", 0.0);

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
      RCLCPP_ERROR(get_logger(), "Parameter 'pcd_file' must point to a saved map.pcd");
      return false;
    }
    if (poses_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'poses_file' must point to the matching poses.txt");
      return false;
    }
    if (remove_self_points_ && patches_dir_.empty()) {
      RCLCPP_ERROR(
        get_logger(), "Parameter 'patches_dir' is required when remove_self_points is true");
      return false;
    }
    if (resolution_ <= 0.0 || z_min_ >= z_max_ || voxel_leaf_size_ <= 0.0 ||
      radius_search_ <= 0.0 || min_neighbors_ <= 0 || map_padding_ < 0.0 ||
      unobserved_value_ < -1 || unobserved_value_ > 100 || free_space_radius_ <= 0.0 ||
      trajectory_clear_radius_ < 0.0 || trajectory_clear_radius_ > free_space_radius_ ||
      self_min_x_ >= self_max_x_ || self_min_y_ >= self_max_y_ ||
      self_min_z_ >= self_max_z_)
    {
      RCLCPP_ERROR(get_logger(), "Invalid map projection parameters");
      return false;
    }
    return true;
  }

  struct TrajectoryPoint
  {
    std::string patch_name;
    double x;
    double y;
    double z;
    double qw;
    double qx;
    double qy;
    double qz;
  };

  bool loadTrajectory(std::vector<TrajectoryPoint> & trajectory) const
  {
    std::ifstream input(poses_file_);
    if (!input.is_open()) {
      RCLCPP_ERROR(get_logger(), "Could not open poses file: %s", poses_file_.c_str());
      return false;
    }

    std::string patch_name;
    double x;
    double y;
    double z;
    double qw;
    double qx;
    double qy;
    double qz;
    while (input >> patch_name >> x >> y >> z >> qw >> qx >> qy >> qz) {
      trajectory.push_back({patch_name, x, y, z, qw, qx, qy, qz});
    }

    if (trajectory.empty()) {
      RCLCPP_ERROR(get_logger(), "No valid poses found in: %s", poses_file_.c_str());
      return false;
    }
    return true;
  }

  bool loadSourceCloud(
    const std::vector<TrajectoryPoint> & trajectory,
    pcl::PointCloud<pcl::PointXYZ>::Ptr source) const
  {
    if (!remove_self_points_) {
      if (pcl::io::loadPCDFile(pcd_file_, *source) < 0) {
        RCLCPP_ERROR(get_logger(), "Could not load PCD file: %s", pcd_file_.c_str());
        return false;
      }
      return true;
    }

    size_t input_points = 0;
    size_t removed_points = 0;
    for (const auto & pose : trajectory) {
      const std::string patch_path = patches_dir_ + "/" + pose.patch_name;
      pcl::PointCloud<pcl::PointXYZ> body_cloud;
      if (pcl::io::loadPCDFile(patch_path, body_cloud) < 0) {
        RCLCPP_ERROR(get_logger(), "Could not load patch: %s", patch_path.c_str());
        return false;
      }

      input_points += body_cloud.size();
      pcl::PointCloud<pcl::PointXYZ> cleaned_body_cloud;
      cleaned_body_cloud.reserve(body_cloud.size());
      for (const auto & point : body_cloud.points) {
        const bool inside_self_box =
          point.x >= self_min_x_ && point.x <= self_max_x_ &&
          point.y >= self_min_y_ && point.y <= self_max_y_ &&
          point.z >= self_min_z_ && point.z <= self_max_z_;
        if (inside_self_box) {
          ++removed_points;
        } else {
          cleaned_body_cloud.push_back(point);
        }
      }

      Eigen::Quaternionf rotation(
        static_cast<float>(pose.qw), static_cast<float>(pose.qx),
        static_cast<float>(pose.qy), static_cast<float>(pose.qz));
      rotation.normalize();
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
      transform.linear() = rotation.toRotationMatrix();
      transform.translation() = Eigen::Vector3f(
        static_cast<float>(pose.x), static_cast<float>(pose.y), static_cast<float>(pose.z));

      pcl::PointCloud<pcl::PointXYZ> world_cloud;
      pcl::transformPointCloud(cleaned_body_cloud, world_cloud, transform);
      *source += world_cloud;
    }

    RCLCPP_INFO(
      get_logger(), "Removed %zu self points from %zu patch points (%.1f%%)",
      removed_points, input_points,
      input_points == 0 ? 0.0 : 100.0 * static_cast<double>(removed_points) /
      static_cast<double>(input_points));
    return !source->empty();
  }

  bool buildMap()
  {
    std::vector<TrajectoryPoint> trajectory;
    if (!loadTrajectory(trajectory)) {
      return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    if (!loadSourceCloud(trajectory, source)) {
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
    for (const auto & pose : trajectory) {
      x_min = std::min(x_min, pose.x - free_space_radius_);
      x_max = std::max(x_max, pose.x + free_space_radius_);
      y_min = std::min(y_min, pose.y - free_space_radius_);
      y_max = std::max(y_max, pose.y + free_space_radius_);
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

    const auto mark_free_disk = [this](double world_x, double world_y, double radius) {
        const int center_x = static_cast<int>(std::floor(
          (world_x - grid_.info.origin.position.x) / resolution_));
        const int center_y = static_cast<int>(std::floor(
          (world_y - grid_.info.origin.position.y) / resolution_));
        const int radius_cells = static_cast<int>(std::ceil(radius / resolution_));
        const double radius_squared = radius * radius;

        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
          for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            const int mx = center_x + dx;
            const int my = center_y + dy;
            if (mx < 0 || mx >= static_cast<int>(grid_.info.width) ||
              my < 0 || my >= static_cast<int>(grid_.info.height))
            {
              continue;
            }
            const double cell_x = grid_.info.origin.position.x +
              (static_cast<double>(mx) + 0.5) * resolution_;
            const double cell_y = grid_.info.origin.position.y +
              (static_cast<double>(my) + 0.5) * resolution_;
            const double offset_x = cell_x - world_x;
            const double offset_y = cell_y - world_y;
            if (offset_x * offset_x + offset_y * offset_y <= radius_squared) {
              grid_.data[static_cast<size_t>(my) * grid_.info.width + mx] = 0;
            }
          }
        }
      };

    const double interpolation_step = resolution_ * 0.5;
    const auto mark_trajectory =
      [&trajectory, &mark_free_disk, interpolation_step](double radius) {
        mark_free_disk(trajectory.front().x, trajectory.front().y, radius);
        for (size_t i = 1; i < trajectory.size(); ++i) {
          const double dx = trajectory[i].x - trajectory[i - 1].x;
          const double dy = trajectory[i].y - trajectory[i - 1].y;
          const double distance = std::hypot(dx, dy);
          const int steps = std::max(
            1, static_cast<int>(std::ceil(distance / interpolation_step)));
          for (int step = 1; step <= steps; ++step) {
            const double ratio = static_cast<double>(step) / static_cast<double>(steps);
            mark_free_disk(
              trajectory[i - 1].x + ratio * dx,
              trajectory[i - 1].y + ratio * dy,
              radius);
          }
        }
      };

    mark_trajectory(free_space_radius_);

    // Obstacles are written after free space so a trajectory corridor can never erase a wall.
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

    const size_t occupied_before_trajectory_clear = static_cast<size_t>(std::count(
      grid_.data.begin(), grid_.data.end(), static_cast<int8_t>(100)));
    if (trajectory_clear_radius_ > 0.0) {
      // The swept robot footprint is confirmed free even if ground/self artifacts project into it.
      mark_trajectory(trajectory_clear_radius_);
    }

    const size_t free_cells = static_cast<size_t>(std::count(
      grid_.data.begin(), grid_.data.end(), static_cast<int8_t>(0)));
    const size_t occupied_cells = static_cast<size_t>(std::count(
      grid_.data.begin(), grid_.data.end(), static_cast<int8_t>(100)));

    RCLCPP_INFO(
      get_logger(),
      "Generated %u x %u map from %zu / %zu PCD points and %zu poses: "
      "%zu free, %zu occupied cells, %zu trajectory artifacts cleared",
      grid_.info.width, grid_.info.height, filtered->size(), source->size(),
      trajectory.size(), free_cells, occupied_cells,
      occupied_before_trajectory_clear - occupied_cells);
    return true;
  }

  void publishMap()
  {
    grid_.header.stamp = now();
    grid_.header.frame_id = frame_id_;
    map_pub_->publish(grid_);
  }

  std::string pcd_file_;
  std::string poses_file_;
  std::string patches_dir_;
  bool remove_self_points_{};
  double self_min_x_{};
  double self_max_x_{};
  double self_min_y_{};
  double self_max_y_{};
  double self_min_z_{};
  double self_max_z_{};
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
  double free_space_radius_{};
  double trajectory_clear_radius_{};
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
