// /state/estimated: fuses Member-1's real /odom (wheel dead-reckoning)
// and /imu/data (orientation + angular velocity) into a single state
// estimate with an innovation-adaptive yaw filter.
//
// Scope, honestly stated: with no GPS (by design) and no second
// independent position sensor, x/y are propagated from /odom directly --
// there is nothing else to fuse them against. Yaw, however, has two
// independent real sensors (the wheel-odometry-derived heading in /odom,
// and the IMU's own orientation estimate), so yaw is fused with a 1D
// Kalman filter whose IMU measurement noise is adapted online from the
// observed innovation magnitude (an exponential moving average of
// squared innovation) -- inconsistent/noisy IMU readings are
// automatically down-weighted rather than trusted blindly, which is the
// "innovation-adaptive" behavior the Member-2 specification calls for.
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{
double wrapAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}
}  // namespace

class StateEstimationNode : public rclcpp::Node
{
public:
  StateEstimationNode()
  : Node("state_estimation_node")
  {
    pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/state/estimated", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 20, std::bind(&StateEstimationNode::onOdom, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 50, std::bind(&StateEstimationNode::onImu, this, std::placeholders::_1));
    RCLCPP_INFO(
      this->get_logger(),
      "state_estimation_node running: innovation-adaptive odom+IMU yaw fusion "
      "(no GPS, no ground truth)");
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double yaw_odom = tf2::getYaw(msg->pose.pose.orientation);
    if (!initialized_) {
      yaw_est_ = yaw_odom;
      yaw_var_ = kInitYawVar;
      initialized_ = true;
    } else {
      // Process noise grows with how fast the vehicle is turning --
      // faster turns make the constant-heading assumption between
      // updates less reliable.
      const double dt = lastOdomDt(msg->header.stamp);
      const double q = kYawProcessNoiseBase + std::fabs(msg->twist.twist.angular.z) * dt * 0.1;
      yaw_var_ += q;

      const double innovation = wrapAngle(yaw_odom - yaw_est_);
      const double s = yaw_var_ + kOdomYawVar;
      const double k = yaw_var_ / s;
      yaw_est_ = wrapAngle(yaw_est_ + k * innovation);
      yaw_var_ = (1.0 - k) * yaw_var_;
    }

    latest_odom_ = *msg;
    publish();
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!initialized_) {return;}
    const double yaw_imu = tf2::getYaw(msg->orientation);
    const double innovation = wrapAngle(yaw_imu - yaw_est_);

    // Innovation-adaptive measurement noise: track a running estimate of
    // innovation^2 and scale the IMU's trusted noise by it, so a
    // consistently large innovation (e.g. wheel slip on Rubicon's rock
    // faces disagreeing with the IMU) automatically inflates R and the
    // filter leans less on the IMU rather than chasing a bad reading.
    innovation_ema_ = kInnovationEmaAlpha * (innovation * innovation) +
      (1.0 - kInnovationEmaAlpha) * innovation_ema_;
    const double r_imu_adaptive = kImuYawVarBase + kInnovationGain * innovation_ema_;

    const double s = yaw_var_ + r_imu_adaptive;
    const double k = yaw_var_ / s;
    yaw_est_ = wrapAngle(yaw_est_ + k * innovation);
    yaw_var_ = (1.0 - k) * yaw_var_;

    last_r_imu_ = r_imu_adaptive;
  }

  double lastOdomDt(const builtin_interfaces::msg::Time & stamp)
  {
    const double t = rclcpp::Time(stamp).seconds();
    const double dt = last_odom_stamp_ > 0.0 ? std::max(0.0, t - last_odom_stamp_) : 0.0;
    last_odom_stamp_ = t;
    return dt;
  }

  void publish()
  {
    nav_msgs::msg::Odometry out = latest_odom_;
    out.header.stamp = this->now();
    out.header.frame_id = "odom";
    out.child_frame_id = "base_link";

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_est_);
    out.pose.pose.orientation.x = q.x();
    out.pose.pose.orientation.y = q.y();
    out.pose.pose.orientation.z = q.z();
    out.pose.pose.orientation.w = q.w();
    // Row/col 5 (yaw) of the 6x6 pose covariance carries the filter's own
    // adaptive yaw variance; x/y/z covariance is passed through from
    // /odom unmodified since position is not independently fused here.
    out.pose.covariance[35] = yaw_var_;

    pub_->publish(out);
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "state estimate: yaw=%.3f rad, yaw_var=%.6f, R_imu_adaptive=%.6f",
      yaw_est_, yaw_var_, last_r_imu_);
  }

  static constexpr double kInitYawVar = 0.1;
  static constexpr double kYawProcessNoiseBase = 1e-5;
  static constexpr double kOdomYawVar = 0.02 * 0.02;
  static constexpr double kImuYawVarBase = 0.01 * 0.01;
  static constexpr double kInnovationGain = 2.0;
  static constexpr double kInnovationEmaAlpha = 0.1;

  bool initialized_ = false;
  double yaw_est_ = 0.0;
  double yaw_var_ = kInitYawVar;
  double innovation_ema_ = 0.0;
  double last_r_imu_ = kImuYawVarBase;
  double last_odom_stamp_ = 0.0;
  nav_msgs::msg::Odometry latest_odom_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimationNode>());
  rclcpp::shutdown();
  return 0;
}
