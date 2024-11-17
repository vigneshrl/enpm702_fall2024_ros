#include <rclcpp/rclcpp.hpp>
#include "bot_publisher.hpp"

int main(int argc, char **argv)
{
    // init
    rclcpp::init(argc, argv);
    // node
    auto node = std::make_shared<BotPublisher>("bot_publisher");
    // spin
    rclcpp::spin(node);
    // cleanup
    rclcpp::shutdown();
}