#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using namespace std::chrono_literals;

class CameraInfoTestPublisher : public rclcpp::Node
{
public:
  CameraInfoTestPublisher()
  : Node("camera_info_test_publisher")
  {
    left_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/left/camera_info", 10);

    right_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/right/camera_info", 10);

    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&CameraInfoTestPublisher::publish_camera_info, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Camera calibration publisher running: f=500 px, baseline=0.12 m");
  }

private:
  sensor_msgs::msg::CameraInfo make_left_info(
    const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::CameraInfo info;

    info.header.stamp = stamp;
    info.header.frame_id = "left_camera";

    info.width = 640;
    info.height = 480;

    info.distortion_model = "plumb_bob";

    // Zero distortion for the synthetic rectified test.
    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    // Intrinsic matrix K.
    info.k = {
      500.0, 0.0, 320.0,
      0.0, 500.0, 240.0,
      0.0, 0.0, 1.0
    };

    // Rectification matrix.
    info.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };

    // Left projection matrix.
    info.p = {
      500.0, 0.0, 320.0, 0.0,
      0.0, 500.0, 240.0, 0.0,
      0.0, 0.0, 1.0, 0.0
    };

    return info;
  }

  sensor_msgs::msg::CameraInfo make_right_info(
    const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::CameraInfo info;

    info.header.stamp = stamp;
    info.header.frame_id = "right_camera";

    info.width = 640;
    info.height = 480;

    info.distortion_model = "plumb_bob";

    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    info.k = {
      500.0, 0.0, 320.0,
      0.0, 500.0, 240.0,
      0.0, 0.0, 1.0
    };

    info.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };

    // Right projection matrix with baseline = 0.12 m.
    info.p = {
      500.0, 0.0, 320.0, -60.0,
      0.0, 500.0, 240.0, 0.0,
      0.0, 0.0, 1.0, 0.0
    };

    return info;
  }

  void publish_camera_info()
  {
    const auto stamp = this->get_clock()->now();

    auto left_info = make_left_info(stamp);
    auto right_info = make_right_info(stamp);

    left_pub_->publish(left_info);
    right_pub_->publish(right_info);
  }

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CameraInfoTestPublisher>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
