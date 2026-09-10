#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::chrono_literals;

class TerrainTestPublisher : public rclcpp::Node
{
public:
  TerrainTestPublisher()
  : Node("terrain_test_publisher")
  {

    this->declare_parameter<bool>("roughness_test", false);
    roughness_test_ =
      this->get_parameter("roughness_test").as_bool();

this->declare_parameter<bool>("obstacle_test", true);
obstacle_test_ =
  this->get_parameter("obstacle_test").as_bool();

    publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/points2", 10);

    timer_ = this->create_wall_timer(
      500ms,
      std::bind(
        &TerrainTestPublisher::publish_cloud,
        this));

    RCLCPP_INFO(
      this->get_logger(),
      "Controlled terrain publisher running");

    RCLCPP_INFO(
      this->get_logger(),
      "Roughness test: %s",
      roughness_test_ ? "ENABLED" : "DISABLED");

    RCLCPP_INFO(
      this->get_logger(),
      "Ground slope: %.2f degrees",
      ground_slope_deg_);

RCLCPP_INFO(
  this->get_logger(),
  "Obstacle test: %s",
  obstacle_test_ ? "ENABLED" : "DISABLED");

    RCLCPP_INFO(
      this->get_logger(),
      "Obstacle: x=[%.2f, %.2f], y=[%.2f, %.2f], height=%.2f m",
      obstacle_x_min_,
      obstacle_x_max_,
      obstacle_y_min_,
      obstacle_y_max_,
      obstacle_height_);
  }

private:
  void publish_cloud()
  {
    std::vector<float> points;

    // ------------------------------------------------------------
    // Ground plane
    //
    // z = z0 + x * tan(theta)
    //
    // This creates a plane tilted around the Y axis.
    // ------------------------------------------------------------

const double effective_slope_deg =
  roughness_test_ ? 0.0 : ground_slope_deg_;

const double theta =
  effective_slope_deg * M_PI / 180.0;
    const double slope =
      std::tan(theta);

    for (double y = -2.0; y <= 2.0; y += 0.04) {
      for (double x = 1.0; x <= 5.0; x += 0.04) {

        double z =
          ground_z_ + slope * (x - 1.0);

        if (roughness_test_) {
          // Alternate Z within each grid cell rather than
          // between cells. Each cell receives both +A and -A
          // samples, giving expected sigma ~= A.
          const int sample_index =
            static_cast<int>(
              std::round((x - 1.0) / 0.04)) +
            static_cast<int>(
              std::round((y + 2.0) / 0.04));

          const double roughness_offset =
            (sample_index % 2 == 0)
              ? roughness_amplitude_
              : -roughness_amplitude_;

          z += roughness_offset;
        }
        points.push_back(
          static_cast<float>(x));

        points.push_back(
          static_cast<float>(y));

        points.push_back(
          static_cast<float>(z));
      }
    }

    // ------------------------------------------------------------
    // Elevated obstacle
    //
    // The obstacle sits above the tilted ground plane.
    // RANSAC should still select the dominant ground plane.
    // ------------------------------------------------------------

    if (obstacle_test_) {
  for (double y = obstacle_y_min_;
         y <= obstacle_y_max_;
         y += 0.04)
    {
      for (double x = obstacle_x_min_;
           x <= obstacle_x_max_;
           x += 0.04)
      {
        const double ground_z =
          ground_z_ + slope * (x - 1.0);

        const double obstacle_top =
          ground_z + obstacle_height_;

        points.push_back(
          static_cast<float>(x));

        points.push_back(
          static_cast<float>(y));

        points.push_back(
          static_cast<float>(obstacle_top));
      }
    }
}
    const std::size_t point_count =
      points.size() / 3;

    sensor_msgs::msg::PointCloud2 msg;

    msg.header.stamp =
      this->get_clock()->now();

    msg.header.frame_id =
      "left_camera";

    msg.height = 1;
    msg.width =
      static_cast<std::uint32_t>(point_count);

    msg.is_bigendian = false;
    msg.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(msg);

    modifier.setPointCloud2FieldsByString(
      1,
      "xyz");

    modifier.resize(point_count);

    sensor_msgs::PointCloud2Iterator<float> iter_x(
      msg,
      "x");

    sensor_msgs::PointCloud2Iterator<float> iter_y(
      msg,
      "y");

    sensor_msgs::PointCloud2Iterator<float> iter_z(
      msg,
      "z");

    for (std::size_t i = 0;
         i < point_count;
         ++i,
         ++iter_x,
         ++iter_y,
         ++iter_z)
    {
      *iter_x = points[3 * i];
      *iter_y = points[3 * i + 1];
      *iter_z = points[3 * i + 2];
    }

    publisher_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Published controlled terrain cloud: %zu points",
      point_count);
  }

  // --------------------------------------------------------------
  // Controlled validation parameters
  // --------------------------------------------------------------

  const double ground_slope_deg_ = 10.0;
  const double ground_z_ = 2.0;

  const double obstacle_x_min_ = 2.5;
  const double obstacle_x_max_ = 3.0;

  const double obstacle_y_min_ = -0.5;
  const double obstacle_y_max_ = 0.5;

  const double obstacle_height_ = 0.50;

  bool roughness_test_ = false;
bool obstacle_test_ = true;

  // Controlled roughness validation:
  // equal +A / -A height offsets give population
  // standard deviation approximately A.
  const double roughness_amplitude_ = 0.05;
  const double roughness_period_ = 0.20;

  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<TerrainTestPublisher>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
