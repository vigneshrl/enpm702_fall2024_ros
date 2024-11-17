#include "p_controller_demo_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProportionalControllerDemoNode>());
    rclcpp::shutdown();
}