#include "qos_subscriber_demo_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/int64.hpp>

void QoSSubscriberDemoNode::receive_int(const example_interfaces::msg::Int64::SharedPtr msg) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Received: " << msg->data);
}
