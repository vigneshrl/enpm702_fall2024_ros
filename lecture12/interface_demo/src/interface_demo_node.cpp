#include "interface_demo_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // For converting quaternion to yaw
#include <enpm702_msgs/msg/bot_status.hpp>
#include <rclcpp/rclcpp.hpp>

                    void
                    InterfaceDemoNode::publish_bot_status() {
    if (bot_model_ != 0) { // If burger, or waffle, or waffle_pi
        bot_status_msg_.model = bot_model_;
        bot_status_msg_.velocities.at(0) = linear_x_;
        bot_status_msg_.velocities.at(1) = angular_z_;
        bot_status_msg_.pose.x = position_x_;
        bot_status_msg_.pose.y = position_y_;
        bot_status_msg_.pose.theta = theta_;
        bot_status_publisher_->publish(bot_status_msg_);
        RCLCPP_INFO(this->get_logger(), "Publishing BotStatus message");
    }
}

void InterfaceDemoNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Retrieve linear.x and angular.z velocities
    linear_x_ = msg->twist.twist.linear.x;
    angular_z_ = msg->twist.twist.angular.z;

    // Retrieve position x and y
    position_x_ = msg->pose.pose.position.x;
    position_y_ = msg->pose.pose.position.y;

    // Retrieve orientation as quaternion and convert to yaw
    tf2::Quaternion quaternion;
    tf2::fromMsg(msg->pose.pose.orientation, quaternion);
    double roll, pitch;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, theta_);

    // Print the retrieved values
    RCLCPP_INFO(this->get_logger(), "Linear Velocity (x): %.2f, Angular Velocity (z): %.2f", linear_x_, angular_z_);
    RCLCPP_INFO(this->get_logger(), "Position (x, y): (%.2f, %.2f), Yaw: %.2f", position_x_, position_y_, theta_);
}