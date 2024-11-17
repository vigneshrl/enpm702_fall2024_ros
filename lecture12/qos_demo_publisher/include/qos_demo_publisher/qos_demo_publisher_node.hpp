#pragma once

#include "rclcpp/rclcpp.hpp"
#include <example_interfaces/msg/int64.hpp>

class QoSDemoPublisherNode : public rclcpp::Node
{
public:
    QoSDemoPublisherNode() : Node("qos_demo_publisher_node")
    {
        // Initialize the publisher
        rclcpp::QoS pub_qos(10);   // keep last 10 messages
        // pub_qos.keep_all();
        pub_qos.reliable();        // reliable reliability
        pub_qos.transient_local(); // transient local durability

        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("qos_demo", pub_qos);

        // // initialize the subscriber
        // rclcpp::QoS sub_qos(10);   // keep last 10 messages
        // sub_qos.reliable();        // reliable reliability
        // sub_qos.transient_local(); // transient local durability
        // subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
        //     "qos_demo", sub_qos, std::bind(&QoSExampleNode::receive_int, this, std::placeholders::_1));

        // Initialize the timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&QoSDemoPublisherNode::publish_int, this));

        RCLCPP_INFO(this->get_logger(), "\033[38;5;214m== QoS Demo Publisher Node Started ==\033[0m");
    }

private:
    int counter_{0};
    void publish_int();
    // void receive_int(const example_interfaces::msg::Int64::SharedPtr msg);
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    // rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
};
