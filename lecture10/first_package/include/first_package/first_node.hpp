#include <rclcpp/rclcpp.hpp>
#include <chrono>
using namespace std::chrono_literals; // Enables direct use of literals

class FirstNode : public rclcpp::Node
{
public:
    FirstNode(std::string node_name, int counter) : rclcpp::Node(node_name), counter_{counter}
    {
        // RCLCPP_INFO_STREAM(this->get_logger(), "Hello");
        log_timer_ = this->create_wall_timer(
            500ms,
            std::bind(&FirstNode::print_message_cb, this));
    }

private:
    rclcpp::TimerBase::SharedPtr log_timer_;
    int counter_;
    void print_message_cb();
};