#include <algorithm>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

class GatingSupervisorNode : public rclcpp::Node
{
public:
  GatingSupervisorNode()
  : Node("gating_supervisor_node")
  {
    this->declare_parameter<double>(
      "entropy_threshold", 0.30);

    entropy_threshold_ =
      this->get_parameter(
        "entropy_threshold").as_double();

    entropy_sub_ =
      this->create_subscription<std_msgs::msg::Float32>(
        "/terrain/entropy",
        10,
        std::bind(
          &GatingSupervisorNode::entropy_callback,
          this,
          std::placeholders::_1));

    decision_pub_ =
      this->create_publisher<std_msgs::msg::String>(
        "/perception/gate_decision",
        10);

    semantic_trigger_pub_ =
      this->create_publisher<std_msgs::msg::Bool>(
        "/perception/semantic_trigger",
        10);

    RCLCPP_INFO(
      this->get_logger(),
      "Gating supervisor is running...");

    RCLCPP_INFO(
      this->get_logger(),
      "Entropy threshold: %.3f",
      entropy_threshold_);
  }

private:
  void entropy_callback(
    const std_msgs::msg::Float32::SharedPtr msg)
  {
    const double entropy =
      static_cast<double>(msg->data);

    if (!std::isfinite(entropy)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Received invalid entropy value");
      return;
    }

    const double bounded_entropy =
      std::clamp(entropy, 0.0, 1.0);

    const bool semantic_required =
      bounded_entropy >= entropy_threshold_;

    std_msgs::msg::String decision_msg;
    std_msgs::msg::Bool trigger_msg;

    if (semantic_required) {
      decision_msg.data = "SEMANTIC_REQUIRED";
      trigger_msg.data = true;
    } else {
      decision_msg.data = "MATH_ONLY";
      trigger_msg.data = false;
    }

    decision_pub_->publish(decision_msg);
    semantic_trigger_pub_->publish(trigger_msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Entropy: %.6f | threshold: %.3f | decision: %s",
      bounded_entropy,
      entropy_threshold_,
      decision_msg.data.c_str());
  }

  double entropy_threshold_{0.30};

  rclcpp::Subscription<
    std_msgs::msg::Float32>::SharedPtr entropy_sub_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr decision_pub_;

  rclcpp::Publisher<
    std_msgs::msg::Bool>::SharedPtr semantic_trigger_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<GatingSupervisorNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
