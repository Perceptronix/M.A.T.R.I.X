#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class StepHeightNode : public rclcpp::Node
{
public:
  StepHeightNode()
  : Node("step_height_node")
  {
    grid_width_ =
      static_cast<int>((x_max_ - x_min_) / resolution_);

    grid_height_ =
      static_cast<int>((y_max_ - y_min_) / resolution_);

    const std::size_t cell_count =
      static_cast<std::size_t>(grid_width_) *
      static_cast<std::size_t>(grid_height_);

    step_height_.assign(
      cell_count,
      std::numeric_limits<float>::quiet_NaN());

    elevation_sub_ =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/terrain/elevation",
        10,
        std::bind(
          &StepHeightNode::elevation_callback,
          this,
          std::placeholders::_1));

    step_height_pub_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/terrain/step_height",
        10);

    RCLCPP_INFO(
      this->get_logger(),
      "Step-height node is running...");

    RCLCPP_INFO(
      this->get_logger(),
      "Grid: %dx%d | resolution: %.2f m",
      grid_width_,
      grid_height_,
      resolution_);
  }

private:
  void elevation_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    const std::size_t cell_count =
      static_cast<std::size_t>(grid_width_) *
      static_cast<std::size_t>(grid_height_);

    if (msg->data.size() != cell_count) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Invalid elevation grid size: %zu (expected %zu)",
        msg->data.size(),
        cell_count);
      return;
    }

    std::fill(
      step_height_.begin(),
      step_height_.end(),
      std::numeric_limits<float>::quiet_NaN());

    std::size_t valid_cells = 0;
    std::size_t computed_cells = 0;

    double max_step = 0.0;
    double sum_step = 0.0;

    for (int iy = 0; iy < grid_height_; ++iy) {

      for (int ix = 0; ix < grid_width_; ++ix) {

        const std::size_t index =
          static_cast<std::size_t>(iy) *
          static_cast<std::size_t>(grid_width_) +
          static_cast<std::size_t>(ix);

        const float current_z = msg->data[index];

        if (!std::isfinite(current_z)) {
          continue;
        }

        ++valid_cells;

        float local_max_step = 0.0f;
        std::size_t valid_neighbors = 0;

        // 4-connected neighborhood:
        // left, right, down, up
        const int dx[4] = {-1, 1, 0, 0};
        const int dy[4] = {0, 0, -1, 1};

        for (int k = 0; k < 4; ++k) {

          const int nx = ix + dx[k];
          const int ny = iy + dy[k];

          if (nx < 0 || nx >= grid_width_ ||
              ny < 0 || ny >= grid_height_)
          {
            continue;
          }

          const std::size_t neighbor_index =
            static_cast<std::size_t>(ny) *
            static_cast<std::size_t>(grid_width_) +
            static_cast<std::size_t>(nx);

          const float neighbor_z =
            msg->data[neighbor_index];

          if (!std::isfinite(neighbor_z)) {
            continue;
          }

          ++valid_neighbors;

          const float height_difference =
            std::fabs(current_z - neighbor_z);

          local_max_step =
            std::max(
              local_max_step,
              height_difference);
        }

        if (valid_neighbors == 0) {
          continue;
        }

        step_height_[index] = local_max_step;

        ++computed_cells;

        sum_step +=
          static_cast<double>(local_max_step);

        max_step =
          std::max(
            max_step,
            static_cast<double>(local_max_step));
      }
    }

    std_msgs::msg::Float32MultiArray output;

    output.layout.dim.resize(2);

    output.layout.dim[0].label = "rows";
    output.layout.dim[0].size =
      static_cast<std::uint32_t>(grid_height_);
    output.layout.dim[0].stride =
      static_cast<std::uint32_t>(cell_count);

    output.layout.dim[1].label = "columns";
    output.layout.dim[1].size =
      static_cast<std::uint32_t>(grid_width_);
    output.layout.dim[1].stride = 1;

    output.data = step_height_;

    step_height_pub_->publish(output);

    const double mean_step =
      computed_cells > 0
      ? sum_step / static_cast<double>(computed_cells)
      : 0.0;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Step height: valid cells=%zu | computed=%zu | "
      "max=%.4f m | mean=%.4f m",
      valid_cells,
      computed_cells,
      max_step,
      mean_step);
  }

  double x_min_ = -3.0;
  double x_max_ = 3.0;
  double y_min_ = -3.0;
  double y_max_ = 3.0;
  double resolution_ = 0.10;

  int grid_width_;
  int grid_height_;

  std::vector<float> step_height_;

  rclcpp::Subscription<
    std_msgs::msg::Float32MultiArray>::SharedPtr
    elevation_sub_;

  rclcpp::Publisher<
    std_msgs::msg::Float32MultiArray>::SharedPtr
    step_height_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<StepHeightNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
