#include "qos_demo_subscriber_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/int64.hpp>

void QoSDemoSubscriberNode::receive_int(const example_interfaces::msg::Int64::SharedPtr msg)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received: " << msg->data);
}

