// /perception/depth: converts the live stereo disparity (produced by the
// stock stereo_image_proc pipeline from Member-1's real left/right cameras)
// into a metric depth image using Z = f*T/d, where f and T (baseline) are
// read from the DisparityImage message itself -- i.e. the REAL calibration
// stereo_image_proc derived from Member-1's live CameraInfo, never a
// hardcoded or synthetic value.
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"
#include "sensor_msgs/msg/image.hpp"

class DepthNode : public rclcpp::Node
{
public:
  DepthNode()
  : Node("depth_node")
  {
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("/perception/depth", 10);
    sub_ = this->create_subscription<stereo_msgs::msg::DisparityImage>(
      "/disparity", rclcpp::SensorDataQoS(),
      std::bind(&DepthNode::callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "depth_node running: Z = f*T/d from live /disparity");
  }

private:
  void callback(const stereo_msgs::msg::DisparityImage::SharedPtr msg)
  {
    const float f = msg->f;
    const float T = msg->t;

    if (!(f > 0.0f) || !(T > 0.0f)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Invalid calibration on /disparity (f=%.3f, T=%.3f); skipping frame", f, T);
      return;
    }

    const sensor_msgs::msg::Image & disp = msg->image;
    sensor_msgs::msg::Image depth;
    depth.header = disp.header;
    depth.height = disp.height;
    depth.width = disp.width;
    depth.encoding = "32FC1";
    depth.is_bigendian = disp.is_bigendian;
    depth.step = disp.width * sizeof(float);
    depth.data.resize(static_cast<size_t>(depth.step) * depth.height);

    const size_t n = static_cast<size_t>(disp.width) * disp.height;
    std::vector<float> d(n);
    std::memcpy(d.data(), disp.data.data(), n * sizeof(float));

    float * out = reinterpret_cast<float *>(depth.data.data());
    size_t valid = 0;
    for (size_t i = 0; i < n; ++i) {
      const float disparity = d[i];
      if (std::isfinite(disparity) && disparity > 0.0f) {
        out[i] = f * T / disparity;   // Z = fB/d, real per-pixel depth
        ++valid;
      } else {
        out[i] = std::numeric_limits<float>::quiet_NaN();
      }
    }

    pub_->publish(depth);
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "depth frame: %zu/%zu valid px (f=%.2f T=%.4f)", valid, n, f, T);
  }

  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthNode>());
  rclcpp::shutdown();
  return 0;
}
