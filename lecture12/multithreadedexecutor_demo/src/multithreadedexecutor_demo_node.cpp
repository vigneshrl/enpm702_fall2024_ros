
#include <multithreadedexecutor_demo_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>  // For rclcpp::sleep_for


void MultiThreadedExecutorDemoNode::timer1_callback() {
    // rclcpp::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO_STREAM(this->get_logger(), output_green("timer1"));
}

void MultiThreadedExecutorDemoNode::timer2_callback() {
    // rclcpp::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO_STREAM(this->get_logger(), output_cyan("timer2"));
}

void MultiThreadedExecutorDemoNode::chatter_callback(const std_msgs::msg::String::SharedPtr msg) {
    // rclcpp::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO_STREAM(this->get_logger(), output_yellow("chatter: " + std::string(msg->data.c_str())));
}
