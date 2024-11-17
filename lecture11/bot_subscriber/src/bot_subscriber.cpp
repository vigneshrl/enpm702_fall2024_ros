#include "bot_subscriber.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/rclcpp.hpp>

void BotSubscriber::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    auto position_x = msg->pose.pose.position.x;
    auto position_y = msg->pose.pose.position.y;
    RCLCPP_INFO_STREAM(this->get_logger(), "Position: [" << position_x << "," << position_y << "]");
}

void BotSubscriber::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    auto beam_0 = msg->ranges[0];
    RCLCPP_INFO_STREAM(this->get_logger(), "Beam 0: " << beam_0);
}