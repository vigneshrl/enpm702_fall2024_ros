#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class BotPublisher : public rclcpp::Node
{
public:
    BotPublisher(std::string node_name) : rclcpp::Node(node_name)
    {
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        auto message = geometry_msgs::msg::Twist();
        // message.linear.x = 1.0;
        message.angular.z = 0.5;
        message.linear.set__x(1.0);
        velocity_publisher_->publish(message);

    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
};