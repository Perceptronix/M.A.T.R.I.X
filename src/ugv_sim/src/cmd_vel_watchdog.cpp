// Deadman/watchdog for /cmd_vel.
//
// The gz-sim-diff-drive-system plugin has no command timeout: it applies
// the last received Twist forever. If the process publishing a stop
// command is ever dropped by DDS discovery (observed during integration
// testing -- a freshly started `ros2 topic pub` can take longer than
// 0.5 s to match the existing bridge subscriber under WSL2, silently
// losing a short burst of stop messages), the vehicle keeps moving
// indefinitely with no operator command in flight. This node publishes a
// zero Twist to /cmd_vel whenever no message (of any kind, including its
// own zero keep-alives) has been seen on /cmd_vel for `timeout` seconds,
// guaranteeing the vehicle fails safe to a stop shortly after any command
// source goes silent, regardless of whether an explicit stop message is
// ever successfully delivered.
//
// C++ port of the original cmd_vel_watchdog.py; behavior and parameter
// name/default are unchanged.

#include <chrono>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class CmdVelWatchdog : public rclcpp::Node
{
public:
  CmdVelWatchdog()
  : Node("cmd_vel_watchdog")
  {
    timeout_sec_ = this->declare_parameter("timeout", 0.3);
    last_rx_ = this->now();

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr) {
        last_rx_ = this->now();
      });
    timer_ = this->create_wall_timer(
      50ms, std::bind(&CmdVelWatchdog::check, this));
  }

private:
  void check()
  {
    const double elapsed = (this->now() - last_rx_).seconds();
    if (elapsed > timeout_sec_) {
      pub_->publish(geometry_msgs::msg::Twist());
    }
  }

  double timeout_sec_;
  rclcpp::Time last_rx_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelWatchdog>());
  rclcpp::shutdown();
  return 0;
}
