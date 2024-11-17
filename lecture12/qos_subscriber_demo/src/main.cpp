#include "qos_subscriber_demo_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QoSSubscriberDemoNode>());
    rclcpp::shutdown();
}