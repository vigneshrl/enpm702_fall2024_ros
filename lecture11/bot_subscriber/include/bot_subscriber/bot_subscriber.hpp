#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <chrono>
using namespace std::chrono_literals; // Enables direct use of literals

class BotSubscriber : public rclcpp::Node
{
public:
    BotSubscriber(std::string node_name) : rclcpp::Node(node_name)
    {
        // initialize odom subscriber
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&BotSubscriber::odom_callback, this, std::placeholders::_1));
        // initialize scan subscriber
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&BotSubscriber::scan_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};