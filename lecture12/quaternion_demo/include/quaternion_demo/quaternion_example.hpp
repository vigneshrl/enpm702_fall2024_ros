#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class QuaternionExampleNode : public rclcpp::Node
{
public:
    QuaternionExampleNode() : Node("quaternion_example")
    {
        // Initialize the publisher
         publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("quaternion_visualization", 10);

        // Initialize the timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&QuaternionExampleNode::publish_pose, this));


        RCLCPP_INFO(this->get_logger(), "\033[38;5;214mQuaternion Node Started\033[0m");
        RCLCPP_INFO_STREAM(this->get_logger(), "================");
        convert_rpy_to_quaternion();
        RCLCPP_INFO_STREAM(this->get_logger(), "================");
        convert_quaternion_to_rpy();
        RCLCPP_INFO_STREAM(this->get_logger(), "================");
        create_pose();
    }

private:
    void convert_rpy_to_quaternion();
    void convert_quaternion_to_rpy();
    void create_pose();
    void publish_pose();
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
