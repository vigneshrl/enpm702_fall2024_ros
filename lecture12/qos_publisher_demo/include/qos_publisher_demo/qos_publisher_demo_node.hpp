#pragma once

#include "rclcpp/rclcpp.hpp"
#include <example_interfaces/msg/int64.hpp>

class QoSPublisherDemoNode : public rclcpp::Node
{
public:
    QoSPublisherDemoNode() : Node("qos_publisher_demo_node")
    {
        // QoS setting for the publisher
        rclcpp::QoS pub_qos(10); // keep last 10 messages
        // pub_qos.keep_all();
        pub_qos.reliable();        // reliable reliability
        pub_qos.transient_local(); // transient local durability
        // Initialize the publisher
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("qos_demo", pub_qos);

        // Initialize the timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&QoSPublisherDemoNode::publish_int, this));

        RCLCPP_INFO(this->get_logger(), "\033[38;5;214m== QoS Publisher Demo Node Started ==\033[0m");
    }

private:
    /**
     * @brief Counter variable used for publishing integer values.
     *
     * Tracks the current count to be published by the `publish_int` method.
     * Initialized to zero.
     */
    int counter_{0};

    /**
     * @brief Publishes the current counter value as an `Int64` message.
     *
     * This function retrieves the current value of the `counter_` variable
     * and publishes it to the associated topic using the `publisher_`.
     */
    void publish_int();

    /**
     * @brief ROS 2 publisher for publishing integer values.
     *
     * Publishes `example_interfaces::msg::Int64` messages containing the value of `counter_`.
     */
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;

    /**
     * @brief ROS 2 timer for scheduling periodic publishing.
     *
     * Triggers the `publish_int` method at regular intervals.
     */
    rclcpp::TimerBase::SharedPtr timer_;
};
