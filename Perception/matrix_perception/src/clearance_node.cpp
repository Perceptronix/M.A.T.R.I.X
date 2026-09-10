#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class ClearanceNode : public rclcpp::Node
{
public:
  ClearanceNode()
  : Node("clearance_node")
  {
    obstacle_sub_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/perception/obstacles",
        rclcpp::SensorDataQoS(),
        std::bind(
          &ClearanceNode::obstacle_callback,
          this,
          std::placeholders::_1));

    clearance_pub_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/terrain/clearance",
        10);

    RCLCPP_INFO(
      this->get_logger(),
      "Clearance node is running...");

    RCLCPP_INFO(
      this->get_logger(),
      "Listening to /perception/obstacles");

    RCLCPP_INFO(
      this->get_logger(),
      "Grid: 60x60 | resolution: %.2f m | X=[%.1f, %.1f] | Y=[%.1f, %.1f]",
      resolution_,
      x_min_,
      x_max_,
      y_min_,
      y_max_);
  }

private:
  void obstacle_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *obstacle_cloud);

    std::size_t valid_obstacles = 0;

    for (const auto & point : obstacle_cloud->points) {
      if (std::isfinite(point.x) &&
          std::isfinite(point.y) &&
          std::isfinite(point.z))
      {
        ++valid_obstacles;
      }
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Obstacles received: %zu total | %zu valid",
      obstacle_cloud->points.size(),
      valid_obstacles);

    std::vector<float> clearance(
      grid_width_ * grid_height_,
      static_cast<float>(max_clearance_));

    std::size_t occupied_cells = 0;

    for (std::size_t row = 0; row < grid_height_; ++row) {
      const double y =
        y_min_ + (static_cast<double>(row) + 0.5) * resolution_;

      for (std::size_t col = 0; col < grid_width_; ++col) {
        const double x =
          x_min_ + (static_cast<double>(col) + 0.5) * resolution_;

        double nearest_distance = max_clearance_;

        for (const auto & point : obstacle_cloud->points) {
          if (!std::isfinite(point.x) ||
              !std::isfinite(point.y) ||
              !std::isfinite(point.z))
          {
            continue;
          }

          if (point.x < x_min_ ||
              point.x > x_max_ ||
              point.y < y_min_ ||
              point.y > y_max_)
          {
            continue;
          }

          const double dx = x - static_cast<double>(point.x);
          const double dy = y - static_cast<double>(point.y);

          const double distance =
            std::sqrt(dx * dx + dy * dy);

          if (distance < nearest_distance) {
            nearest_distance = distance;
          }
        }

        if (nearest_distance < resolution_) {
          ++occupied_cells;
          nearest_distance = 0.0;
        }

        clearance[row * grid_width_ + col] =
          static_cast<float>(nearest_distance);
      }
    }

    std_msgs::msg::Float32MultiArray clearance_msg;

    clearance_msg.layout.dim.resize(2);

    clearance_msg.layout.dim[0].label = "rows";
    clearance_msg.layout.dim[0].size =
      static_cast<std::uint32_t>(grid_height_);
    clearance_msg.layout.dim[0].stride =
      static_cast<std::uint32_t>(grid_width_ * grid_height_);

    clearance_msg.layout.dim[1].label = "columns";
    clearance_msg.layout.dim[1].size =
      static_cast<std::uint32_t>(grid_width_);
    clearance_msg.layout.dim[1].stride =
      static_cast<std::uint32_t>(grid_width_);

    clearance_msg.data = clearance;

    clearance_pub_->publish(clearance_msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Clearance grid: %zux%zu | occupied cells: %zu | "
      "max clearance: %.2f m",
      grid_width_,
      grid_height_,
      occupied_cells,
      max_clearance_);
  }

  static constexpr std::size_t grid_width_ = 60;
  static constexpr std::size_t grid_height_ = 60;

  static constexpr double resolution_ = 0.10;

  static constexpr double x_min_ = -3.0;
  static constexpr double x_max_ =  3.0;

  static constexpr double y_min_ = -3.0;
  static constexpr double y_max_ =  3.0;

  // Finite cap used when no obstacle is nearby.
  static constexpr double max_clearance_ = 6.0;

  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::SharedPtr
    obstacle_sub_;

  rclcpp::Publisher<
    std_msgs::msg::Float32MultiArray>::SharedPtr
    clearance_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<ClearanceNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
