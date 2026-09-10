#include <memory>
#include <cmath>
#include <functional>
#include <limits>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <cstdint>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Dense>

class GroundPlaneNode : public rclcpp::Node
{
public:
  GroundPlaneNode()
  : Node("ground_plane_node")
  {
    pointcloud_sub_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points2",
        rclcpp::SensorDataQoS(),
        std::bind(
          &GroundPlaneNode::pointcloud_callback,
          this,
          std::placeholders::_1));

      obstacle_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/perception/obstacles",
          rclcpp::SensorDataQoS());

slope_pub_ =
  this->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/terrain/slope",
    10);

    RCLCPP_INFO(
      this->get_logger(),
      "Ground plane node is running...");

    RCLCPP_INFO(
      this->get_logger(),
      "Listening to /points2");

    RCLCPP_INFO(
      this->get_logger(),
      "ROI: X=[%.2f, %.2f] Y=[%.2f, %.2f] Z=[%.2f, %.2f]",
      roi_x_min_, roi_x_max_,
      roi_y_min_, roi_y_max_,
      roi_z_min_, roi_z_max_);
  }

private:
  void pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // ------------------------------------------------------------
    // Stage 1: PointCloud2 -> PCL
    // ------------------------------------------------------------

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *cloud);

    const std::size_t total_points = cloud->points.size();

    // ------------------------------------------------------------
    // Stage 2: finite XYZ filtering
    // ------------------------------------------------------------

    pcl::PointCloud<pcl::PointXYZ>::Ptr valid_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto & point : cloud->points) {
      if (std::isfinite(point.x) &&
          std::isfinite(point.y) &&
          std::isfinite(point.z))
      {
        valid_cloud->points.push_back(point);
      }
    }

    const std::size_t valid_points =
      valid_cloud->points.size();

    const std::size_t invalid_points =
      total_points - valid_points;

    // ------------------------------------------------------------
    // Stage 3: ROI filtering
    // ------------------------------------------------------------

    pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto & point : valid_cloud->points) {
      if (point.x < roi_x_min_ ||
          point.x > roi_x_max_ ||
          point.y < roi_y_min_ ||
          point.y > roi_y_max_ ||
          point.z < roi_z_min_ ||
          point.z > roi_z_max_)
      {
        continue;
      }

      roi_cloud->points.push_back(point);
    }

    const std::size_t roi_points =
      roi_cloud->points.size();

    // ------------------------------------------------------------
    // Stage 4: RANSAC plane estimation
    // ------------------------------------------------------------

    pcl::SACSegmentation<pcl::PointXYZ> segmentation;

    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);

    // Temporary validation threshold.
    segmentation.setDistanceThreshold(0.05);

    segmentation.setInputCloud(roi_cloud);

    pcl::PointIndices::Ptr inliers(
      new pcl::PointIndices);

    pcl::ModelCoefficients::Ptr coefficients(
      new pcl::ModelCoefficients);

    segmentation.segment(
      *inliers,
      *coefficients);

    if (inliers->indices.empty() ||
        coefficients->values.size() < 4)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "RANSAC: no valid plane found | ROI points: %zu",
        roi_points);

      return;
    }

    const double ransac_a =
      coefficients->values[0];

    const double ransac_b =
      coefficients->values[1];

    const double ransac_c =
      coefficients->values[2];

    const double ransac_d =
      coefficients->values[3];

    const double ransac_normal_magnitude =
      std::sqrt(
        ransac_a * ransac_a +
        ransac_b * ransac_b +
        ransac_c * ransac_c);

    // ------------------------------------------------------------
    // Stage 5: SVD / covariance refinement
    //
    // RANSAC provides robust inliers.
    //
    // The inliers are centered around their centroid and the
    // covariance matrix is computed.
    //
    // The eigenvector corresponding to the smallest eigenvalue
    // is the refined plane normal.
    // ------------------------------------------------------------

    if (inliers->indices.size() < 3)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "SVD: insufficient RANSAC inliers: %zu",
        inliers->indices.size());

      return;
    }

    Eigen::Vector3d centroid =
      Eigen::Vector3d::Zero();

    for (const int index : inliers->indices) {

      const auto & point =
        roi_cloud->points[index];

      centroid.x() +=
        static_cast<double>(point.x);

      centroid.y() +=
        static_cast<double>(point.y);

      centroid.z() +=
        static_cast<double>(point.z);
    }

    centroid /=
      static_cast<double>(
        inliers->indices.size());

    Eigen::Matrix3d covariance =
      Eigen::Matrix3d::Zero();

    for (const int index : inliers->indices) {

      const auto & point =
        roi_cloud->points[index];

      const Eigen::Vector3d centered(
        static_cast<double>(point.x) - centroid.x(),
        static_cast<double>(point.y) - centroid.y(),
        static_cast<double>(point.z) - centroid.z());

      covariance +=
        centered * centered.transpose();
    }

    covariance /=
      static_cast<double>(
        inliers->indices.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>
      eigen_solver(covariance);

    if (eigen_solver.info() != Eigen::Success)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "SVD: covariance eigen decomposition failed");

      return;
    }

    const Eigen::Vector3d eigenvalues =
      eigen_solver.eigenvalues();

    // Eigenvalues are ordered from smallest to largest.
    // Therefore column 0 is the eigenvector associated
    // with the smallest eigenvalue.

    Eigen::Vector3d refined_normal =
      eigen_solver.eigenvectors().col(0);

    const double refined_normal_magnitude =
      refined_normal.norm();

    if (!std::isfinite(refined_normal_magnitude) ||
        refined_normal_magnitude < 1e-12)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "SVD: invalid refined normal");

      return;
    }

    refined_normal.normalize();

    // ------------------------------------------------------------
    // Keep SVD normal direction consistent with RANSAC normal.
    // ------------------------------------------------------------

    const Eigen::Vector3d ransac_normal(
      ransac_a,
      ransac_b,
      ransac_c);

    if (refined_normal.dot(ransac_normal) < 0.0)
    {
      refined_normal = -refined_normal;
    }

    // ------------------------------------------------------------
    // Stage 6: surface slope
    //
    // For normalized normal n = [nx, ny, nz]:
    //
    // theta = acos(|nz|)
    //
    // This gives the angle between the surface normal and
    // the vertical axis.
    // ------------------------------------------------------------

    const double nz_abs =
      std::clamp(
        std::abs(refined_normal.z()),
        0.0,
        1.0);

    const double slope_rad =
      std::acos(nz_abs);

    const double slope_deg =
      slope_rad * 180.0 / M_PI;

std_msgs::msg::Float32MultiArray slope_msg;

const int grid_width = 60;
const int grid_height = 60;

slope_msg.layout.dim.resize(2);

slope_msg.layout.dim[0].label = "height";
slope_msg.layout.dim[0].size =
  static_cast<std::uint32_t>(grid_height);
slope_msg.layout.dim[0].stride =
  static_cast<std::uint32_t>(grid_width * grid_height);

slope_msg.layout.dim[1].label = "width";
slope_msg.layout.dim[1].size =
  static_cast<std::uint32_t>(grid_width);
slope_msg.layout.dim[1].stride =
  static_cast<std::uint32_t>(grid_width);

slope_msg.data.resize(
  static_cast<std::size_t>(grid_width * grid_height),
  static_cast<float>(slope_deg));

slope_pub_->publish(slope_msg);

    // ------------------------------------------------------------
    // Stage 7: ground / non-ground classification
    //
    // For each ROI point:
    //
    // distance = |n . (p - centroid)|
    //
    // Because n is normalized, this is the perpendicular
    // distance from the point to the refined plane.
    // ------------------------------------------------------------

    std::size_t ground_points = 0;
    std::size_t non_ground_points = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

    double max_ground_distance = 0.0;

    double min_non_ground_distance =
      std::numeric_limits<double>::max();

    for (const auto & point : roi_cloud->points) {

      const Eigen::Vector3d point_vector(
        static_cast<double>(point.x),
        static_cast<double>(point.y),
        static_cast<double>(point.z));

      const double distance =
        std::abs(
          refined_normal.dot(
            point_vector - centroid));

      if (distance <= ground_distance_threshold_) {

        ++ground_points;

        if (distance > max_ground_distance) {
          max_ground_distance = distance;
        }

      } else {

        ++non_ground_points;
        obstacle_cloud->points.push_back(point);

        if (distance < min_non_ground_distance) {
          min_non_ground_distance = distance;
        }
      }
    }

    obstacle_cloud->width =
      static_cast<std::uint32_t>(obstacle_cloud->points.size());
    obstacle_cloud->height = 1;
    obstacle_cloud->is_dense = true;

    sensor_msgs::msg::PointCloud2 obstacle_msg;
    pcl::toROSMsg(*obstacle_cloud, obstacle_msg);
    obstacle_msg.header = msg->header;
    obstacle_pub_->publish(obstacle_msg);

    // ------------------------------------------------------------
    // Validation output
    // ------------------------------------------------------------

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "PointCloud: %zu total | %zu valid | %zu ROI | %zu invalid",
      total_points,
      valid_points,
      roi_points,
      invalid_points);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "RANSAC plane: %.6fx + %.6fy + %.6fz + %.6f = 0 | "
      "inliers: %zu | |normal|: %.6f",
      ransac_a,
      ransac_b,
      ransac_c,
      ransac_d,
      inliers->indices.size(),
      ransac_normal_magnitude);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "SVD centroid: [%.6f, %.6f, %.6f]",
      centroid.x(),
      centroid.y(),
      centroid.z());

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "SVD eigenvalues: [%.9e, %.9e, %.9e]",
      eigenvalues[0],
      eigenvalues[1],
      eigenvalues[2]);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "SVD refined normal: [%.6f, %.6f, %.6f] | |normal|: %.6f",
      refined_normal.x(),
      refined_normal.y(),
      refined_normal.z(),
      refined_normal.norm());

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Surface slope: %.6f degrees",
      slope_deg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Classification: %zu ground | %zu non-ground | "
      "threshold: %.3f m",
      ground_points,
      non_ground_points,
      ground_distance_threshold_);

    if (non_ground_points > 0) {

      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Classification distances: "
        "max ground: %.6f m | min non-ground: %.6f m",
        max_ground_distance,
        min_non_ground_distance);
    }
  }

  // --------------------------------------------------------------
  // Temporary ROI parameters for the current controlled fixture.
  //
  // /points2 is currently expressed in the left_camera frame.
  // --------------------------------------------------------------

  const double roi_x_min_ = -3.0;
  const double roi_x_max_ =  3.0;

  const double roi_y_min_ = -3.0;
  const double roi_y_max_ =  3.0;

  const double roi_z_min_ =  0.5;
  const double roi_z_max_ =  8.0;

  // Temporary validation threshold.
  const double ground_distance_threshold_ = 0.05;

  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::SharedPtr
    pointcloud_sub_;

  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::SharedPtr
    obstacle_pub_;

rclcpp::Publisher<
  std_msgs::msg::Float32MultiArray>::SharedPtr
  slope_pub_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<GroundPlaneNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
