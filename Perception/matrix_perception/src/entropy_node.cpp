#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class EntropyNode : public rclcpp::Node
{
public:
  EntropyNode()
  : Node("entropy_node")
  {
    local_width_ =
      this->declare_parameter<int>("local_width", 20);

    local_height_ =
      this->declare_parameter<int>("local_height", 20);

    bin_count_ =
      this->declare_parameter<int>("bin_count", 10);

    if (local_width_ < 1 || local_width_ > static_cast<int>(grid_width_)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid local_width; using 20");
      local_width_ = 20;
    }

    if (local_height_ < 1 || local_height_ > static_cast<int>(grid_height_)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid local_height; using 20");
      local_height_ = 20;
    }

    if (bin_count_ < 2 || bin_count_ > 100) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid bin_count; using 10");
      bin_count_ = 10;
    }

    variance_sub_ =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/terrain/elevation_variance",
        10,
        std::bind(
          &EntropyNode::variance_callback,
          this,
          std::placeholders::_1));

    entropy_pub_ =
      this->create_publisher<std_msgs::msg::Float32>(
        "/terrain/entropy",
        10);

    RCLCPP_INFO(
      this->get_logger(),
      "Entropy node is running...");

    RCLCPP_INFO(
      this->get_logger(),
      "Using localized geometric uncertainty entropy");

    RCLCPP_INFO(
      this->get_logger(),
      "Window: %dx%d | bins: %d",
      local_width_,
      local_height_,
      bin_count_);
  }

private:
  void variance_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != grid_width_ * grid_height_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Invalid variance grid size: %zu",
        msg->data.size());
      return;
    }

    const std::size_t local_width =
      static_cast<std::size_t>(local_width_);

    const std::size_t local_height =
      static_cast<std::size_t>(local_height_);

    const std::size_t x_start =
      (grid_width_ - local_width) / 2;

    const std::size_t y_start =
      (grid_height_ - local_height) / 2;

    std::vector<double> local_variance;

    local_variance.reserve(
      local_width * local_height);

    for (std::size_t y = y_start;
         y < y_start + local_height;
         ++y)
    {
      for (std::size_t x = x_start;
           x < x_start + local_width;
           ++x)
      {
        const std::size_t index =
          y * grid_width_ + x;

        const double variance =
          static_cast<double>(msg->data[index]);

        if (std::isfinite(variance) &&
            variance >= 0.0)
        {
          local_variance.push_back(variance);
        }
      }
    }

    std_msgs::msg::Float32 entropy_msg;

    if (local_variance.size() < 2) {
      entropy_msg.data = 0.0f;
      entropy_pub_->publish(entropy_msg);

      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Insufficient valid cells for entropy");
      return;
    }

    const double min_variance =
      *std::min_element(
        local_variance.begin(),
        local_variance.end());

    const double max_variance =
      *std::max_element(
        local_variance.begin(),
        local_variance.end());

    const double range =
      max_variance - min_variance;

    const std::size_t bin_count =
      static_cast<std::size_t>(bin_count_);

    std::vector<std::size_t> histogram(
      bin_count,
      0);

    if (range <= 1e-12) {
      histogram[0] = local_variance.size();
    } else {
      for (const double variance : local_variance) {
        const double normalized =
          (variance - min_variance) / range;

        std::size_t bin =
          static_cast<std::size_t>(
            std::floor(
              normalized *
              static_cast<double>(bin_count)));

        if (bin >= bin_count) {
          bin = bin_count - 1;
        }

        histogram[bin]++;
      }
    }

    const double sample_count =
      static_cast<double>(
        local_variance.size());

    double entropy = 0.0;

    for (const std::size_t count : histogram) {
      if (count == 0) {
        continue;
      }

      const double probability =
        static_cast<double>(count) /
        sample_count;

      entropy -=
        probability *
        std::log(probability);
    }

    const double max_entropy =
      std::log(
        static_cast<double>(bin_count));

    const double normalized_entropy =
      max_entropy > 0.0
      ? std::clamp(
          entropy / max_entropy,
          0.0,
          1.0)
      : 0.0;

    entropy_msg.data =
      static_cast<float>(
        normalized_entropy);

    entropy_pub_->publish(entropy_msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Geometric entropy: %.6f | "
      "valid local cells: %zu | "
      "variance range: %.9f to %.9f m^2",
      normalized_entropy,
      local_variance.size(),
      min_variance,
      max_variance);
  }

  static constexpr std::size_t grid_width_ = 60;
  static constexpr std::size_t grid_height_ = 60;

  int local_width_{20};
  int local_height_{20};
  int bin_count_{10};

  rclcpp::Subscription<
    std_msgs::msg::Float32MultiArray>::SharedPtr
    variance_sub_;

  rclcpp::Publisher<
    std_msgs::msg::Float32>::SharedPtr
    entropy_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<EntropyNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
