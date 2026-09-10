#include <cstdint>
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

class RoughnessNode : public rclcpp::Node
{
public:
  RoughnessNode()
  : Node("roughness_node")
  {
    grid_width_ =
      static_cast<int>((x_max_ - x_min_) / resolution_);

    grid_height_ =
      static_cast<int>((y_max_ - y_min_) / resolution_);

    pointcloud_sub_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points2",
        rclcpp::SensorDataQoS(),
        std::bind(
          &RoughnessNode::pointcloud_callback,
          this,
          std::placeholders::_1));

    roughness_pub_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/terrain/roughness",
        10);

    RCLCPP_INFO(
      this->get_logger(),
      "Roughness node is running...");

    RCLCPP_INFO(
      this->get_logger(),
      "Grid: %dx%d | resolution=%.2f m",
      grid_width_,
      grid_height_,
      resolution_);


  }

private:
  void pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *cloud);

    const std::size_t cell_count =
      static_cast<std::size_t>(grid_width_) *
      static_cast<std::size_t>(grid_height_);

    std::vector<double> height_sum(
      cell_count, 0.0);

    std::vector<double> height_squared_sum(
      cell_count, 0.0);

    std::vector<std::size_t> point_count(
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

      const double z =
        static_cast<double>(point.z);

      height_sum[index] += z;
      height_squared_sum[index] += z * z;
      point_count[index]++;
    }

    std::size_t occupied_cells = 0;

    double min_roughness =
      std::numeric_limits<double>::max();

    double max_roughness =
      std::numeric_limits<double>::lowest();

    double roughness_sum = 0.0;

    for (std::size_t i = 0; i < cell_count; ++i) {

      if (point_count[i] < 2) {
        continue;
      }

      const double n =
        static_cast<double>(point_count[i]);

      const double mean_z =
        height_sum[i] / n;

      // Population variance:
      //
      // variance =
      // E[z^2] - E[z]^2
      //
      const double variance =
        (height_squared_sum[i] / n) -
        (mean_z * mean_z);

      const double safe_variance =
        std::max(variance, 0.0);

      const double roughness =
        std::sqrt(safe_variance);

      ++occupied_cells;

      min_roughness =
        std::min(min_roughness, roughness);

      max_roughness =
        std::max(max_roughness, roughness);

      roughness_sum += roughness;
    }

    const double mean_roughness =
      occupied_cells > 0
      ? roughness_sum /
        static_cast<double>(occupied_cells)
      : 0.0;

    std_msgs::msg::Float32MultiArray roughness_msg;

    roughness_msg.layout.dim.resize(2);

    roughness_msg.layout.dim[0].label = "height";
    roughness_msg.layout.dim[0].size =
      static_cast<std::uint32_t>(grid_height_);
    roughness_msg.layout.dim[0].stride =
      static_cast<std::uint32_t>(grid_width_ * grid_height_);

    roughness_msg.layout.dim[1].label = "width";
    roughness_msg.layout.dim[1].size =
      static_cast<std::uint32_t>(grid_width_);
    roughness_msg.layout.dim[1].stride =
      static_cast<std::uint32_t>(grid_width_);

    roughness_msg.data.resize(cell_count);

    for (std::size_t i = 0; i < cell_count; ++i) {
      roughness_msg.data[i] =
        std::numeric_limits<float>::quiet_NaN();
    }

    for (std::size_t i = 0; i < cell_count; ++i) {
      if (point_count[i] >= 2) {
        const double n =
          static_cast<double>(point_count[i]);

        const double mean_z =
          height_sum[i] / n;

        const double variance =
          (height_squared_sum[i] / n) -
          (mean_z * mean_z);

        const double safe_variance =
          std::max(variance, 0.0);

        roughness_msg.data[i] =
          static_cast<float>(std::sqrt(safe_variance));
      }
    }

    roughness_pub_->publish(roughness_msg);


    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Roughness grid: %dx%d | cells: %zu | "
      "occupied: %zu",
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
        "Roughness: min=%.9f m | "
        "max=%.9f m | mean=%.9f m",
        min_roughness,
        max_roughness,
        mean_roughness);
    }
  }

  const double x_min_ = -3.0;
  const double x_max_ =  3.0;

  const double y_min_ = -3.0;
  const double y_max_ =  3.0;

  const double resolution_ = 0.10;

  int grid_width_ = 0;
  int grid_height_ = 0;

  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::SharedPtr
    pointcloud_sub_;

  rclcpp::Publisher<
    std_msgs::msg::Float32MultiArray>::SharedPtr
    roughness_pub_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<RoughnessNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
