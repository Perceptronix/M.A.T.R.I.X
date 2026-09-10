#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class EntropyTestPublisher : public rclcpp::Node
{
public:
  EntropyTestPublisher()
  : Node("entropy_test_publisher")
  {
    this->declare_parameter<bool>("mixed_test", false);
    mixed_test_ = this->get_parameter("mixed_test").as_bool();

    pub_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/terrain/elevation_variance",
        10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(
        &EntropyTestPublisher::publish_test,
        this));

    RCLCPP_INFO(
      this->get_logger(),
      "Entropy test publisher: %s",
      mixed_test_ ? "MIXED VARIANCE" : "UNIFORM VARIANCE");
  }

private:
  void publish_test()
  {
    std_msgs::msg::Float32MultiArray msg;

    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "height";
    msg.layout.dim[0].size = 60;
    msg.layout.dim[0].stride = 3600;
    msg.layout.dim[1].label = "width";
    msg.layout.dim[1].size = 60;
    msg.layout.dim[1].stride = 60;

    msg.data.resize(3600);

    for (std::size_t y = 0; y < 60; ++y) {
      for (std::size_t x = 0; x < 60; ++x) {

        const std::size_t index = y * 60 + x;

        if (!mixed_test_) {
          // Uniform uncertainty.
          msg.data[index] = 0.001f;
        } else {
          // Four distinct uncertainty regions inside
          // the central 20x20 entropy window.
          if (x >= 20 && x < 25) {
            msg.data[index] = 0.001f;
          } else if (x >= 25 && x < 30) {
            msg.data[index] = 0.003f;
          } else if (x >= 30 && x < 35) {
            msg.data[index] = 0.006f;
          } else if (x >= 35 && x < 40) {
            msg.data[index] = 0.010f;
          } else {
            msg.data[index] = 0.001f;
          }
        }
      }
    }

    pub_->publish(msg);
  }

  bool mixed_test_{false};

  rclcpp::Publisher<
    std_msgs::msg::Float32MultiArray>::SharedPtr pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<EntropyTestPublisher>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
