#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class StereoTestPublisher : public rclcpp::Node
{
public:
  StereoTestPublisher()
  : Node("stereo_test_publisher")
  {
    left_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/left/image_raw", 10);

    right_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/right/image_raw", 10);

    left_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/left/camera_info", 10);

    right_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/right/camera_info", 10);

    const std::string base =
      std::string(std::getenv("HOME")) +
      "/matrix_ws/test_data/stereo/";

    left_image_ = cv::imread(base + "left.png", cv::IMREAD_GRAYSCALE);
    right_image_ = cv::imread(base + "right.png", cv::IMREAD_GRAYSCALE);

    if (left_image_.empty() || right_image_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load stereo test images.");
      throw std::runtime_error("Stereo test images could not be loaded.");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded stereo images: %dx%d",
      left_image_.cols,
      left_image_.rows);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&StereoTestPublisher::publish_stereo, this));
  }

private:
  sensor_msgs::msg::CameraInfo make_camera_info(
    const rclcpp::Time & stamp,
    const std::string & frame_id,
    bool right_camera)
  {
    sensor_msgs::msg::CameraInfo info;

    info.header.stamp = stamp;
    info.header.frame_id = frame_id;

    info.width = 640;
    info.height = 480;

    info.distortion_model = "plumb_bob";

    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    // Intrinsic matrix K
    info.k = {
      500.0, 0.0, 320.0,
      0.0, 500.0, 240.0,
      0.0, 0.0, 1.0
    };

    // Rectification matrix R
    info.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };

    // Projection matrix P
    if (right_camera) {
      info.p = {
        500.0, 0.0, 320.0, -60.0,
        0.0, 500.0, 240.0, 0.0,
        0.0, 0.0, 1.0, 0.0
      };
    } else {
      info.p = {
        500.0, 0.0, 320.0, 0.0,
        0.0, 500.0, 240.0, 0.0,
        0.0, 0.0, 1.0, 0.0
      };
    }

    info.binning_x = 0;
    info.binning_y = 0;

    return info;
  }

  void publish_stereo()
  {
    // ONE timestamp shared by all four messages.
    const auto stamp = this->get_clock()->now();

    auto left_msg =
      cv_bridge::CvImage(
        std_msgs::msg::Header(),
        "mono8",
        left_image_)
      .toImageMsg();

    auto right_msg =
      cv_bridge::CvImage(
        std_msgs::msg::Header(),
        "mono8",
        right_image_)
      .toImageMsg();

    left_msg->header.stamp = stamp;
    right_msg->header.stamp = stamp;

    left_msg->header.frame_id = "left_camera";
    right_msg->header.frame_id = "right_camera";

    auto left_info =
      make_camera_info(stamp, "left_camera", false);

    auto right_info =
      make_camera_info(stamp, "right_camera", true);

    left_pub_->publish(*left_msg);
    right_pub_->publish(*right_msg);

    left_info_pub_->publish(left_info);
    right_info_pub_->publish(right_info);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;

  cv::Mat left_image_;
  cv::Mat right_image_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<StereoTestPublisher>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Exception: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
