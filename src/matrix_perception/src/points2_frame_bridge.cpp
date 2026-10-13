// Frame bridge, this integration's addition (not part of the upstream
// nodes). The upstream ground_plane_node/elevation_grid_node/
// roughness_node/clearance_node all index a fixed-size local grid
// directly off an incoming PointCloud2's x/y fields, with no internal TF
// transform -- they were evidently developed/tested against a synthetic
// fixture already in a REP-103-style frame (X-forward, Y-left, Z-up).
// Member-1's real stereo_image_proc output (/points2) is in the camera's
// OPTICAL frame instead (Z-forward, X-right, Y-down), which would
// silently corrupt every one of those nodes' ROI/grid semantics if fed
// directly. This node performs the one missing, genuinely necessary
// transform -- via real, live TF (never Gazebo ground truth) -- and
// republishes on a new topic; the four consumer nodes are wired to it by
// ROS remapping in perception.launch.py, with no source changes to any
// upstream file.
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class Points2FrameBridge : public rclcpp::Node
{
public:
  Points2FrameBridge()
  : Node("points2_frame_bridge"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points2/base_link", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points2", rclcpp::SensorDataQoS(),
      std::bind(&Points2FrameBridge::onCloud, this, std::placeholders::_1));
    RCLCPP_INFO(
      this->get_logger(),
      "points2_frame_bridge: /points2 -> /points2/base_link (target_frame=%s)",
      target_frame_.c_str());
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
        target_frame_, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(0.2));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "TF unavailable for frame '%s': %s", msg->header.frame_id.c_str(), ex.what());
      return;
    }
    sensor_msgs::msg::PointCloud2 out;
    tf2::doTransform(*msg, out, tf);
    out.header.frame_id = target_frame_;
    pub_->publish(out);
  }

  std::string target_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Points2FrameBridge>());
  rclcpp::shutdown();
  return 0;
}
