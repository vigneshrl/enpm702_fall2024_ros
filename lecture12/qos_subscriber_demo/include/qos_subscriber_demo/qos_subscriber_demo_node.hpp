#pragma once

#include <example_interfaces/msg/int64.hpp>
#include <string>
#include "color_utils.hpp"

#include "rclcpp/rclcpp.hpp"

class QoSSubscriberDemoNode : public rclcpp::Node {
   public:
    QoSSubscriberDemoNode()
        : Node("qos_subscriber_demo_node") {
        // initialize the subscriber
        rclcpp::QoS sub_qos(10);  // keep last 10 messages
        // sub_qos.keep_all();
        sub_qos.reliable();         // reliable reliability
        sub_qos.transient_local();  // transient local durability
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "qos_demo", sub_qos, std::bind(&QoSSubscriberDemoNode::receive_int, this, std::placeholders::_1));

        RCLCPP_INFO_STREAM(this->get_logger(), output_yellow("== QoS Demo Subscriber Node Started =="));
    }

   private:
    /**
     * @brief Callback function to handle received messages of type Int64.
     *
     * This function is called whenever a message is received on the subscribed topic.
     * It processes the message of type `example_interfaces::msg::Int64`.
     *
     * @param msg A shared pointer to the received message.
     */
    void receive_int(const example_interfaces::msg::Int64::SharedPtr msg);

    /**
     * @brief Subscriber to topics publishing messages of type Int64.
     *
     * This subscriber listens to messages of type `example_interfaces::msg::Int64`.
     * It invokes the `receive_int` callback function whenever a new message is received.
     */
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
};
