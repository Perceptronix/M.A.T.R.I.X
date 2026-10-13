#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class TraversabilityCostmapNode : public rclcpp::Node
{
public:
  TraversabilityCostmapNode()
  : Node("traversability_costmap_node")
  {
    slope_sub_ =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/terrain/slope",
        10,
        std::bind(
          &TraversabilityCostmapNode::slope_callback,
          this,
          std::placeholders::_1));

    roughness_sub_ =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/terrain/roughness",
        10,
        std::bind(
          &TraversabilityCostmapNode::roughness_callback,
          this,
          std::placeholders::_1));

    step_sub_ =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/terrain/step_height",
        10,
        std::bind(
          &TraversabilityCostmapNode::step_callback,
          this,
          std::placeholders::_1));

    clearance_sub_ =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/terrain/clearance",
        10,
        std::bind(
          &TraversabilityCostmapNode::clearance_callback,
          this,
          std::placeholders::_1));

    variance_sub_ =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/terrain/elevation_variance",
        10,
        std::bind(
          &TraversabilityCostmapNode::variance_callback,
          this,
          std::placeholders::_1));

    costmap_pub_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/terrain/costmap",
        10);

    RCLCPP_INFO(
      this->get_logger(),
      "Traversability costmap node is running...");

    RCLCPP_INFO(
      this->get_logger(),
      "Grid: %zux%zu | resolution: %.2f m",
      grid_width_,
      grid_height_,
      resolution_);

    RCLCPP_INFO(
      this->get_logger(),
      "Weights: slope=%.2f roughness=%.2f step=%.2f "
      "clearance=%.2f variance=%.2f",
      weight_slope_,
      weight_roughness_,
      weight_step_,
      weight_clearance_,
      weight_variance_);
  }

private:

  static constexpr std::size_t grid_width_ = 60;
  static constexpr std::size_t grid_height_ = 60;
  static constexpr std::size_t cell_count_ =
    grid_width_ * grid_height_;

  static constexpr double resolution_ = 0.10;

  static constexpr double x_min_ = -3.0;
  static constexpr double y_min_ = -3.0;

  // Development weights.
  static constexpr double weight_slope_ = 0.25;
  static constexpr double weight_roughness_ = 0.20;
  static constexpr double weight_step_ = 0.25;
  static constexpr double weight_clearance_ = 0.20;
  static constexpr double weight_variance_ = 0.10;

  // Normalization thresholds.
  static constexpr double slope_limit_deg_ = 30.0;
  static constexpr double roughness_limit_m_ = 0.10;
  static constexpr double step_limit_m_ = 0.30;
  static constexpr double clearance_limit_m_ = 2.0;
  static constexpr double variance_limit_m2_ = 0.01;

  void slope_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!valid_grid(msg)) {
      return;
    }

    slope_ = msg->data;
    slope_ready_ = true;

    publish_costmap();
  }

  void roughness_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!valid_grid(msg)) {
      return;
    }

    roughness_ = msg->data;
    roughness_ready_ = true;

    publish_costmap();
  }

  void step_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!valid_grid(msg)) {
      return;
    }

    step_height_ = msg->data;
    step_ready_ = true;

    publish_costmap();
  }

  void clearance_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!valid_grid(msg)) {
      return;
    }

    clearance_ = msg->data;
    clearance_ready_ = true;

    publish_costmap();
  }

  void variance_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!valid_grid(msg)) {
      return;
    }

    variance_ = msg->data;
    variance_ready_ = true;

    publish_costmap();
  }

  bool valid_grid(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
  {
    return msg->data.size() == cell_count_;
  }

  double normalize_positive(
    double value,
    double limit) const
  {
    if (!std::isfinite(value)) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    if (value <= 0.0) {
      return 0.0;
    }

    return std::clamp(value / limit, 0.0, 1.0);
  }

  double normalize_clearance(
    double clearance) const
  {
    if (!std::isfinite(clearance)) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    if (clearance >= clearance_limit_m_) {
      return 0.0;
    }

    if (clearance <= 0.0) {
      return 1.0;
    }

    return std::clamp(
      1.0 - clearance / clearance_limit_m_,
      0.0,
      1.0);
  }

  void publish_costmap()
  {
    if (!slope_ready_ ||
        !roughness_ready_ ||
        !step_ready_ ||
        !clearance_ready_ ||
        !variance_ready_)
    {
      return;
    }

    nav_msgs::msg::OccupancyGrid costmap;

    costmap.header.stamp = this->now();
    costmap.header.frame_id = "base_link";

    costmap.info.resolution =
      static_cast<float>(resolution_);

    costmap.info.width =
      static_cast<std::uint32_t>(grid_width_);

    costmap.info.height =
      static_cast<std::uint32_t>(grid_height_);

    costmap.info.origin.position.x = x_min_;
    costmap.info.origin.position.y = y_min_;
    costmap.info.origin.position.z = 0.0;

    costmap.info.origin.orientation.x = 0.0;
    costmap.info.origin.orientation.y = 0.0;
    costmap.info.origin.orientation.z = 0.0;
    costmap.info.origin.orientation.w = 1.0;

    costmap.data.resize(cell_count_);

    std::size_t valid_cells = 0;
    std::size_t unknown_cells = 0;

    double min_cost = 1.0;
    double max_cost = 0.0;
    double cost_sum = 0.0;

    for (std::size_t i = 0; i < cell_count_; ++i) {

      const double slope =
        static_cast<double>(slope_[i]);

      const double roughness =
        static_cast<double>(roughness_[i]);

      const double step =
        static_cast<double>(step_height_[i]);

      const double clearance =
        static_cast<double>(clearance_[i]);

      const double variance =
        static_cast<double>(variance_[i]);

      const double c_slope =
        normalize_positive(
          std::abs(slope),
          slope_limit_deg_);

      const double c_roughness =
        normalize_positive(
          roughness,
          roughness_limit_m_);

      const double c_step =
        normalize_positive(
          step,
          step_limit_m_);

      const double c_clearance =
        normalize_clearance(clearance);

      const double c_variance =
        normalize_positive(
          variance,
          variance_limit_m2_);

      if (!std::isfinite(c_slope) ||
          !std::isfinite(c_roughness) ||
          !std::isfinite(c_step) ||
          !std::isfinite(c_clearance) ||
          !std::isfinite(c_variance))
      {
        costmap.data[i] = -1;
        ++unknown_cells;
        continue;
      }

      const double total_cost =
        weight_slope_ * c_slope +
        weight_roughness_ * c_roughness +
        weight_step_ * c_step +
        weight_clearance_ * c_clearance +
        weight_variance_ * c_variance;

      const double bounded_cost =
        std::clamp(total_cost, 0.0, 1.0);

      const int occupancy =
        static_cast<int>(
          std::lround(bounded_cost * 100.0));

      costmap.data[i] =
        static_cast<std::int8_t>(
          std::clamp(occupancy, 0, 100));

      ++valid_cells;

      min_cost =
        std::min(min_cost, bounded_cost);

      max_cost =
        std::max(max_cost, bounded_cost);

      cost_sum += bounded_cost;
    }

    costmap_pub_->publish(costmap);

    const double mean_cost =
      valid_cells > 0
      ? cost_sum / static_cast<double>(valid_cells)
      : 0.0;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Costmap: valid=%zu | unknown=%zu | "
      "cost range=[%.3f, %.3f] | mean=%.3f",
      valid_cells,
      unknown_cells,
      min_cost,
      max_cost,
      mean_cost);
  }

  std::mutex mutex_;

  std::vector<float> slope_;
  std::vector<float> roughness_;
  std::vector<float> step_height_;
  std::vector<float> clearance_;
  std::vector<float> variance_;

  bool slope_ready_ = false;
  bool roughness_ready_ = false;
  bool step_ready_ = false;
  bool clearance_ready_ = false;
  bool variance_ready_ = false;

  rclcpp::Subscription<
    std_msgs::msg::Float32MultiArray>::SharedPtr slope_sub_;

  rclcpp::Subscription<
    std_msgs::msg::Float32MultiArray>::SharedPtr roughness_sub_;

  rclcpp::Subscription<
    std_msgs::msg::Float32MultiArray>::SharedPtr step_sub_;

  rclcpp::Subscription<
    std_msgs::msg::Float32MultiArray>::SharedPtr clearance_sub_;

  rclcpp::Subscription<
    std_msgs::msg::Float32MultiArray>::SharedPtr variance_sub_;

  rclcpp::Publisher<
    nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<TraversabilityCostmapNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
