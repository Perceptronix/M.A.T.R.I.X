#include <memory>
#include <cmath>
#include <limits>
#include <vector>
#include <functional>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class ElevationGridNode : public rclcpp::Node
{
public:
  ElevationGridNode()
  : Node("elevation_grid_node")
  {
    grid_width_ =
      static_cast<int>((x_max_ - x_min_) / resolution_);

    grid_height_ =
      static_cast<int>((y_max_ - y_min_) / resolution_);

    if (grid_width_ <= 0 || grid_height_ <= 0) {
      throw std::runtime_error("Invalid elevation grid dimensions");
    }

    const std::size_t cell_count =
      static_cast<std::size_t>(grid_width_) *
      static_cast<std::size_t>(grid_height_);

    elevation_mean_.assign(
      cell_count, 0.0);

    elevation_variance_.assign(
      cell_count, 0.0);

    observation_count_.assign(
      cell_count, 0);

    initialized_.assign(
      cell_count, false);

elevation_pub_ =
  this->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/terrain/elevation", 10);

variance_pub_ =
  this->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/terrain/elevation_variance", 10);

    pointcloud_sub_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points2",
        rclcpp::SensorDataQoS(),
        std::bind(
          &ElevationGridNode::pointcloud_callback,
          this,
          std::placeholders::_1));



    RCLCPP_INFO(
      this->get_logger(),
      "Bayesian elevation grid node is running...");

    RCLCPP_INFO(
      this->get_logger(),
      "Grid: X=[%.2f, %.2f] Y=[%.2f, %.2f] resolution=%.2f m",
      x_min_, x_max_,
      y_min_, y_max_,
      resolution_);

    RCLCPP_INFO(
      this->get_logger(),
      "Measurement variance: %.6f m^2",
      measurement_variance_);
  }

private:
  void bayesian_update(
    const std::size_t index,
    const double measurement_mean)
  {
    if (!initialized_[index]) {

      elevation_mean_[index] =
        measurement_mean;

      elevation_variance_[index] =
        measurement_variance_;

      observation_count_[index] = 1;
      initialized_[index] = true;

      return;
    }

    const double prior_mean =
      elevation_mean_[index];

    const double prior_variance =
      elevation_variance_[index];

    const double denominator =
      prior_variance + measurement_variance_;

    if (!std::isfinite(denominator) ||
        denominator <= 1e-12)
    {
      return;
    }

    const double kalman_gain =
      prior_variance / denominator;

    const double posterior_mean =
      prior_mean +
      kalman_gain *
      (measurement_mean - prior_mean);

    const double posterior_variance =
      (1.0 - kalman_gain) *
      prior_variance;

    elevation_mean_[index] =
      posterior_mean;

    elevation_variance_[index] =
      std::max(
        posterior_variance,
        minimum_variance_);

    observation_count_[index]++;
  }

  void pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *cloud);

    const std::size_t cell_count =
      static_cast<std::size_t>(grid_width_) *
      static_cast<std::size_t>(grid_height_);

    // Measurements accumulated for this sensor frame.
    std::vector<double> frame_height_sum(
      cell_count, 0.0);

    std::vector<std::size_t> frame_point_count(
      cell_count, 0);

    std::size_t valid_points = 0;
    std::size_t roi_points = 0;

    for (const auto & point : cloud->points) {

      if (!std::isfinite(point.x) ||
          !std::isfinite(point.y) ||
          !std::isfinite(point.z))
      {
        continue;
      }

      ++valid_points;

      if (point.x < x_min_ ||
          point.x >= x_max_ ||
          point.y < y_min_ ||
          point.y >= y_max_)
      {
        continue;
      }

      ++roi_points;

      const int ix =
        static_cast<int>(
          std::floor(
            (point.x - x_min_) /
            resolution_));

      const int iy =
        static_cast<int>(
          std::floor(
            (point.y - y_min_) /
            resolution_));

      if (ix < 0 || ix >= grid_width_ ||
          iy < 0 || iy >= grid_height_)
      {
        continue;
      }

      const std::size_t index =
        static_cast<std::size_t>(iy) *
        static_cast<std::size_t>(grid_width_) +
        static_cast<std::size_t>(ix);

      frame_height_sum[index] +=
        static_cast<double>(point.z);

      frame_point_count[index]++;
    }

    std::size_t occupied_cells = 0;

    double min_elevation =
      std::numeric_limits<double>::max();

    double max_elevation =
      std::numeric_limits<double>::lowest();

    double elevation_sum = 0.0;

    double variance_sum = 0.0;

    std::size_t total_observations = 0;

    for (std::size_t i = 0; i < cell_count; ++i) {

      if (frame_point_count[i] == 0) {
        continue;
      }

      const double measurement_mean =
        frame_height_sum[i] /
        static_cast<double>(
          frame_point_count[i]);

      bayesian_update(
        i,
        measurement_mean);

      ++occupied_cells;

      min_elevation =
        std::min(
          min_elevation,
          elevation_mean_[i]);

      max_elevation =
        std::max(
          max_elevation,
          elevation_mean_[i]);

      elevation_sum +=
        elevation_mean_[i];

      variance_sum +=
        elevation_variance_[i];

      total_observations +=
        observation_count_[i];
    }

    const double mean_elevation =
      occupied_cells > 0
      ? elevation_sum /
        static_cast<double>(occupied_cells)
      : 0.0;

    const double mean_variance =
      occupied_cells > 0
      ? variance_sum /
        static_cast<double>(occupied_cells)
      : 0.0;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Bayesian elevation grid: %dx%d | cells: %zu | "
      "observed this frame: %zu",
      grid_width_,
      grid_height_,
      cell_count,
      occupied_cells);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Points: %zu valid | %zu ROI",
      valid_points,
      roi_points);

    if (occupied_cells > 0) {

      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Elevation posterior: min=%.6f m | "
        "max=%.6f m | mean=%.6f m",
        min_elevation,
        max_elevation,
        mean_elevation);

      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Variance posterior: mean=%.9f m^2 | "
        "total observations=%zu",
        mean_variance,
        total_observations);
    }
    // ------------------------------------------------------------
    // Publish Bayesian terrain layers.
    // Row-major layout: [y][x].
    // ------------------------------------------------------------
    std_msgs::msg::Float32MultiArray elevation_msg;
    elevation_msg.layout.dim.resize(2);
    elevation_msg.layout.dim[0].label = "height";
    elevation_msg.layout.dim[0].size = static_cast<std::size_t>(grid_height_);
    elevation_msg.layout.dim[0].stride = static_cast<std::size_t>(grid_width_) * static_cast<std::size_t>(grid_height_);
    elevation_msg.layout.dim[1].label = "width";
    elevation_msg.layout.dim[1].size = static_cast<std::size_t>(grid_width_);
    elevation_msg.layout.dim[1].stride = static_cast<std::size_t>(grid_width_);
    elevation_msg.data.resize(cell_count);

    std_msgs::msg::Float32MultiArray variance_msg;
    variance_msg.layout = elevation_msg.layout;
    variance_msg.data.resize(cell_count);

    for (std::size_t i = 0; i < cell_count; ++i) {
      elevation_msg.data[i] = initialized_[i] ? static_cast<float>(elevation_mean_[i]) : std::numeric_limits<float>::quiet_NaN();
      variance_msg.data[i] = initialized_[i] ? static_cast<float>(elevation_variance_[i]) : std::numeric_limits<float>::quiet_NaN();
    }

    elevation_pub_->publish(elevation_msg);
    variance_pub_->publish(variance_msg);
  }

  // ------------------------------------------------------------
  // Controlled validation grid.
  // ------------------------------------------------------------

  const double x_min_ = -3.0;
  const double x_max_ =  3.0;

  const double y_min_ = -3.0;
  const double y_max_ =  3.0;

  // 10 cm cells.
  const double resolution_ = 0.10;

  // Development assumption for the controlled fixture.
  // This is NOT a real sensor specification.
  const double measurement_variance_ = 0.01;

  // Numerical floor preventing zero posterior variance.
  const double minimum_variance_ = 1e-9;

  int grid_width_ = 0;
  int grid_height_ = 0;

  std::vector<double> elevation_mean_;
  std::vector<double> elevation_variance_;
  std::vector<std::size_t> observation_count_;
  std::vector<bool> initialized_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr elevation_pub_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr variance_pub_;


  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::SharedPtr
    pointcloud_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<ElevationGridNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
