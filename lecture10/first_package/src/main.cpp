#include <rclcpp/rclcpp.hpp>
#include "first_node.hpp"

int main(int argc, char **argv)
{
    // init
    rclcpp::init(argc, argv);
    // node
    auto node = std::make_shared<FirstNode>("hello", 2);
    // spin
    rclcpp::spin(node);
    // cleanup
    rclcpp::shutdown();
}