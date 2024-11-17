#include "quaternion_example.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"



void QuaternionExampleNode::publish_pose()
{
    geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->now();
        pose_stamped.header.frame_id = "map";

        // Set position (optional, just for visualization)
        pose_stamped.pose.position.x = 0.0;
        pose_stamped.pose.position.y = 0.0;
        pose_stamped.pose.position.z = 0.0;

        // Set orientation (quaternion)
        pose_stamped.pose.orientation.x = 0.707;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        pose_stamped.pose.orientation.w = 0.707;

        publisher_->publish(pose_stamped);

        RCLCPP_INFO_STREAM(this->get_logger(), "Published pose with quaternion: "
            << "x: " << pose_stamped.pose.orientation.x << ", "
            << "y: " << pose_stamped.pose.orientation.y << ", "
            << "z: " << pose_stamped.pose.orientation.z << ", "
            << "w: " << pose_stamped.pose.orientation.w);
    }


void QuaternionExampleNode::convert_rpy_to_quaternion()
{
    // Create a quaternion using tf2
    tf2::Quaternion quaternion;
    quaternion.setRPY(1.57, 0, 0); // Roll = 90 degrees, Pitch = 0, Yaw = 0

    // Normalize the quaternion
    quaternion.normalize();

    // Convert tf2 quaternion to geometry_msgs quaternion
    geometry_msgs::msg::Quaternion msg_quaternion;
    msg_quaternion.x = quaternion.x();
    msg_quaternion.y = quaternion.y();
    msg_quaternion.z = quaternion.z();
    msg_quaternion.w = quaternion.w();

    RCLCPP_INFO(this->get_logger(), "Quaternion: [x: %f, y: %f, z: %f, w: %f]", msg_quaternion.x, msg_quaternion.y, msg_quaternion.z, msg_quaternion.w);
}

void QuaternionExampleNode::convert_quaternion_to_rpy()
{
    // Example quaternion (x, y, z, w)
    tf2::Quaternion quaternion(0.706825, 0.0, 0.0, 0.707388); // 90 degrees roll

    // Convert quaternion to roll, pitch, and yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    // Print the results
    RCLCPP_INFO(this->get_logger(), "Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
}

void QuaternionExampleNode::create_pose()
{
    geometry_msgs::msg::Pose pose{};

    // Set position
    pose.position.x = 1.0;
    pose.position.y = 2.0;
    pose.position.z = 3.0;

    // Set orientation using quaternion
    tf2::Quaternion quaternion{};
    quaternion.setRPY(0, 0, 1.57); // Yaw = 90 degrees
    quaternion.normalize();

    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();

    // Log the pose and RPY using RCLCPP_INFO_STREAM
    RCLCPP_INFO_STREAM(this->get_logger(), "Pose Position: [x: " << pose.position.x << ", y: " << pose.position.y << ", z: " << pose.position.z << "]");
    RCLCPP_INFO_STREAM(this->get_logger(), "================");
    RCLCPP_INFO_STREAM(this->get_logger(), "Pose Orientation (Quaternion): [x: " << pose.orientation.x << ", y: " << pose.orientation.y << ", z: " << pose.orientation.z << ", w: " << pose.orientation.w << "]");
}