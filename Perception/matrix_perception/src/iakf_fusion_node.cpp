#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <memory>
#include <sstream>
#include <string>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"

class IakfFusionNode : public rclcpp::Node
{
public:
  IakfFusionNode()
  : Node("iakf_fusion_node")
  {
    dt_ = this->declare_parameter<double>("dt", 0.01);
    window_size_ = this->declare_parameter<int>("window_size", 10);
    process_noise_ = this->declare_parameter<double>("process_noise", 0.001);
    measurement_noise_ =
      this->declare_parameter<double>("measurement_noise", 0.05);

    if (window_size_ < 2) {
      window_size_ = 2;
    }

    x_ = Eigen::Vector3d::Zero();

    P_ =
      Eigen::Matrix3d::Identity() * 1.0;

    Q_ =
      Eigen::Matrix3d::Identity() * process_noise_;

    R_base_ =
      Eigen::Matrix3d::Identity() * measurement_noise_;

    H_ =
      Eigen::Matrix3d::Identity();

    measurement_sub_ =
      this->create_subscription<geometry_msgs::msg::Vector3>(
        "/iakf/measurement",
        10,
        std::bind(
          &IakfFusionNode::measurement_callback,
          this,
          std::placeholders::_1));

    odom_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>(
        "/state/estimated",
        10);

    RCLCPP_INFO(
      this->get_logger(),
      "IAKF fusion node is running...");

    RCLCPP_INFO(
      this->get_logger(),
      "State: [x, y, v] | dt=%.4f | window=%d",
      dt_,
      window_size_);
  }

private:
  void measurement_callback(
    const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    const Eigen::Vector3d z(
      msg->x,
      msg->y,
      msg->z);

    // Constant-velocity prediction.
    Eigen::Matrix3d F =
      Eigen::Matrix3d::Identity();

    F(0, 2) = dt_;

    const Eigen::Vector3d x_pred =
      F * x_;

    const Eigen::Matrix3d P_pred =
      F * P_ * F.transpose() + Q_;

    // Innovation residual:
    // y_k = z_k - H x_k^-
    const Eigen::Vector3d innovation =
      z - H_ * x_pred;

    innovation_window_.push_back(innovation);

    if (innovation_window_.size() >
        static_cast<std::size_t>(window_size_))
    {
      innovation_window_.pop_front();
    }

    // Sample innovation covariance.
    Eigen::Matrix3d C_y =
      Eigen::Matrix3d::Zero();

    for (const auto & residual : innovation_window_) {
      C_y += residual * residual.transpose();
    }

    C_y /=
      static_cast<double>(
        innovation_window_.size());

    // Adaptive measurement covariance:
    // R_adapted =
    // max(R_base, C_y - H P_pred H^T)
    Eigen::Matrix3d R_adapted =
      C_y -
      H_ * P_pred * H_.transpose();

    for (int i = 0; i < 3; ++i) {
      R_adapted(i, i) =
        std::max(
          R_base_(i, i),
          R_adapted(i, i));
    }

    // Keep R numerically usable.
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        if (r != c) {
          R_adapted(r, c) = 0.0;
        }
      }
    }

    const Eigen::Matrix3d S =
      H_ * P_pred * H_.transpose() +
      R_adapted;

    const Eigen::Matrix3d K =
      P_pred *
      H_.transpose() *
      S.inverse();

    // EKF/Kalman update.
    x_ =
      x_pred +
      K * innovation;

    P_ =
      (Eigen::Matrix3d::Identity() -
       K * H_) *
      P_pred;

    // Numerical symmetry.
    P_ =
      0.5 * (P_ + P_.transpose());

    publish_odometry();

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "innovation=[%.4f %.4f %.4f] | "
      "R=[%.5f %.5f %.5f] | "
      "state=[%.4f %.4f %.4f]",
      innovation(0),
      innovation(1),
      innovation(2),
      R_adapted(0, 0),
      R_adapted(1, 1),
      R_adapted(2, 2),
      x_(0),
      x_(1),
      x_(2));
  }

  void publish_odometry()
  {
    nav_msgs::msg::Odometry msg;

    msg.header.stamp =
      this->get_clock()->now();

    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

    msg.pose.pose.position.x = x_(0);
    msg.pose.pose.position.y = x_(1);

    msg.twist.twist.linear.x = x_(2);

    msg.pose.covariance[0] = P_(0, 0);
    msg.pose.covariance[7] = P_(1, 1);
    msg.twist.covariance[0] = P_(2, 2);

    odom_pub_->publish(msg);
  }

  double dt_{0.01};
  int window_size_{10};
  double process_noise_{0.001};
  double measurement_noise_{0.05};

  Eigen::Vector3d x_;
  Eigen::Matrix3d P_;
  Eigen::Matrix3d Q_;
  Eigen::Matrix3d R_base_;
  Eigen::Matrix3d H_;

  std::deque<Eigen::Vector3d> innovation_window_;

  rclcpp::Subscription<
    geometry_msgs::msg::Vector3>::SharedPtr
    measurement_sub_;

  rclcpp::Publisher<
    nav_msgs::msg::Odometry>::SharedPtr
    odom_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<IakfFusionNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
