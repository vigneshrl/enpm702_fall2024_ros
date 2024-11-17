#include "qos_demo_publisher_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/int64.hpp>

// void QoSExampleNode::receive_int(const example_interfaces::msg::Int64::SharedPtr msg)
// {
//     RCLCPP_INFO_STREAM(this->get_logger(), "Message: " << msg->data);
// }

void QoSDemoPublisherNode::publish_int()
{
    auto message = example_interfaces::msg::Int64();
    message.data = counter_;
    publisher_->publish(message);
    RCLCPP_INFO_STREAM(this->get_logger(), "Published: " << message.data);
    counter_++;
}