#pragma once

#include <cstdlib>  // For std::getenv
#include <enpm702_msgs/msg/bot_status.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "color_utils.hpp"
#include "rclcpp/rclcpp.hpp"

class InterfaceDemoNode : public rclcpp::Node {
   public:
    InterfaceDemoNode()
        : Node("interface_demo_node") {
        // Initialize bot_status_msg_
        bot_status_msg_ = enpm702_msgs::msg::BotStatus();

        // Retrieve the environment variable "TURTLEBOT3_MODEL"
        const char *turtlebot_model = std::getenv("TURTLEBOT3_MODEL");
        if (turtlebot_model)  // Check if the pointer is not null
        {
            if (std::strcmp(turtlebot_model, "burger") == 0) {
                // Environment variable is set to "burger"
                bot_model_ = enpm702_msgs::msg::BotStatus::BURGER;
            } else if (std::strcmp(turtlebot_model, "waffle") == 0) {
                // Environment variable is set to "waffle"
                bot_model_ = enpm702_msgs::msg::BotStatus::WAFFLE;
            } else if (std::strcmp(turtlebot_model, "waffle_pi") == 0) {
                // Environment variable is set to "waffle_pi"
                bot_model_ = enpm702_msgs::msg::BotStatus::WAFFLE_PI;
            } else {
                RCLCPP_ERROR_STREAM(this->get_logger(), "== Unknown TURTLEBOT3_MODEL value: " << turtlebot_model << "==");
            }
        } else {
            RCLCPP_ERROR_STREAM(this->get_logger(), "== TURTLEBOT3_MODEL not set ==");
        }

        // Initialize the subscriber to the /odom topic
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&InterfaceDemoNode::odom_callback, this, std::placeholders::_1));

        // Initialize the publisher
        bot_status_publisher_ = this->create_publisher<enpm702_msgs::msg::BotStatus>("interface_demo", 10);

        // Initialize the timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&InterfaceDemoNode::publish_bot_status, this));

        RCLCPP_INFO_STREAM(this->get_logger(), output_yellow("== Interface Demo Node Started =="));
    }

   private:
    /**
     * @brief Publishes the current status of the robot.
     *
     * This function retrieves the current status of the vehicle
     * and publishes it to the associated topic using the `publisher_`.
     *  - The current pose of the robot is retrieved from the topic `odom`.
     *  - The current linear and angular velocities are retrieved from the topic `odom`.
     *  - The current model of the vehicle is retrieved from the environment variable TURTLEBOT3_MODEL.
     */
    void publish_bot_status();

    /**
     * @brief Callback function for processing odometry messages.
     *
     * This function is triggered whenever a new message is published to the `/odom` topic.
     * It retrieves the robot's linear velocity, angular velocity, position, and orientation.
     * The orientation, provided as a quaternion, is converted to yaw.
     *
     * @param msg Shared pointer to the incoming odometry message of type `nav_msgs::msg::Odometry`.
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    /**
     * @brief ROS 2 publisher for publishing the robot status.
     *
     * Publishes `enpm702_msgs::msg::BotStatus` messages.
     */
    rclcpp::Publisher<enpm702_msgs::msg::BotStatus>::SharedPtr bot_status_publisher_;

    /**
     * @brief ROS 2 timer for scheduling periodic publishing.
     *
     * Triggers the `publish_bot_status` method at regular intervals.
     */
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * @brief Model of the Turtlebot.
     *
     * The model is retrieved from the environment variable TURTLEBOT3_MODEL and then converted to unsigned int.
     */
    unsigned bot_model_{0};

    /**
     * @brief Message object to store one message of type enpm702_msgs::msg::BotStatus.
     *
     * The model is retrieved from the environment variable TURTLEBOT3_MODEL and then converted to unsigned int.
     */
    enpm702_msgs::msg::BotStatus bot_status_msg_;

    /**
     * @brief ROS 2 subscription to the /odom topic.
     *
     * This subscription listens to messages of type `nav_msgs::msg::Odometry`
     * published on the `/odom` topic. The data is used to retrieve the robot's
     * linear and angular velocities, position, and orientation (converted to yaw).
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    /**
     * @brief Represents the robot's linear velocity along the x-axis.
     */
    double linear_x_{0.0};

    /**
     * @brief Represents the robot's angular velocity around the z-axis.
     */
    double angular_z_{0.0};

    /**
     * @brief Represents the robot's x-coordinate position in the world frame.
     */
    double position_x_{0.0};

    /**
     * @brief Represents the robot's y-coordinate position in the world frame.
     */
    double position_y_{0.0};

    /**
     * @brief Represents the robot's orientation (yaw) in radians.
     *
     * The yaw is calculated from the quaternion representation of the robot's orientation.
     * It specifies the rotation around the z-axis in the 2D plane.
     */
    double theta_{0.0};
};
