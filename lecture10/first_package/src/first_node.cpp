#include "first_node.hpp"

void FirstNode::print_message_cb()
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Hello, world: " << counter_);
    counter_++;
}
