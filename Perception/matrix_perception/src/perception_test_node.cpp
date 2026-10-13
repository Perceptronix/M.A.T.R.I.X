#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class PerceptionTestNode : public rclcpp::Node
{
public:
    PerceptionTestNode()
        : Node("perception_test_node")
    {
        timer_ = this->create_wall_timer(
            1s,
            std::bind(&PerceptionTestNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        RCLCPP_INFO(
            this->get_logger(),
            "MATRIX perception node is running...");
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PerceptionTestNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
