#pragma once

#include "rclcpp/rclcpp.hpp"
#include <example_interfaces/msg/int64.hpp>

class QoSDemoSubscriberNode : public rclcpp::Node
{
public:
    QoSDemoSubscriberNode() : Node("qos_demo_subscriber_node")
    {
        
        // initialize the subscriber
        rclcpp::QoS sub_qos(10);   // keep last 10 messages
        // sub_qos.keep_all();
        sub_qos.reliable();        // reliable reliability
        sub_qos.transient_local(); // transient local durability
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "qos_demo", sub_qos, std::bind(&QoSDemoSubscriberNode::receive_int, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "\033[38;5;214m== QoS Demo Subscriber Node Started ==\033[0m");
    }

private:

    void receive_int(const example_interfaces::msg::Int64::SharedPtr msg);
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
};
