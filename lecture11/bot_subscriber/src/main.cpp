#include <rclcpp/rclcpp.hpp>
#include "bot_subscriber.hpp"

int main(int argc, char **argv)
{
    // init
    rclcpp::init(argc, argv);
    // node
    auto node = std::make_shared<BotSubscriber>("bot_subscriber");
    // spin
    rclcpp::spin(node);
    // cleanup
    rclcpp::shutdown();
}