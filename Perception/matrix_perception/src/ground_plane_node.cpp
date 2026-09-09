#include <memory>
#include <cmath>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class GroundPlaneNode : public rclcpp::Node
{
public:
  GroundPlaneNode()
  : Node("ground_plane_node")
  {
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points2",
      rclcpp::SensorDataQoS(),
      std::bind(&GroundPlaneNode::pointcloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Ground plane node is running...");
    RCLCPP_INFO(this->get_logger(), "Listening to /points2");
  }

private:
  void pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *cloud);

    std::size_t valid_points = 0;

    for (const auto & point : cloud->points) {
      if (std::isfinite(point.x) &&
          std::isfinite(point.y) &&
          std::isfinite(point.z))
      {
        ++valid_points;
      }
    }

    const std::size_t total_points = cloud->points.size();
    const std::size_t invalid_points = total_points - valid_points;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "PointCloud: %zu total | %zu valid XYZ | %zu invalid",
      total_points,
      valid_points,
      invalid_points);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GroundPlaneNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
