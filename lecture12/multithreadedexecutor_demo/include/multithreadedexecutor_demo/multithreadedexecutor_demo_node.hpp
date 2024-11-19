#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "color_utils.hpp"

class MultiThreadedExecutorDemoNode : public rclcpp::Node {
   public:
    MultiThreadedExecutorDemoNode()
        : Node("multithreaded_executor_demo") {
        // Callback groups
        mutually_exclusive_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        reentrant_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Initialize timer1_
        timer1_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&MultiThreadedExecutorDemoNode::timer1_callback, this),
            reentrant_group_);

        // Initialize timer2_
        timer2_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&MultiThreadedExecutorDemoNode::timer2_callback, this),
            reentrant_group_);

        // Initialize subscriber
        auto options = rclcpp::SubscriptionOptions();
        options.callback_group = mutually_exclusive_group_;
        subscription_ = this->create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&MultiThreadedExecutorDemoNode::chatter_callback, this, std::placeholders::_1), options);

        RCLCPP_INFO_STREAM(this->get_logger(), output_yellow("== multithreaded_executor_demo started =="));
    }

   private:
    /**
     * @brief Callback group that allows reentrant execution of callbacks.
     *
     * This callback group enables multiple callbacks within the group to execute
     * concurrently when used with a MultiThreadedExecutor. It is suitable for
     * callbacks that are independent and do not share resources, such as timers
     * or subscriptions that can run in parallel.
     *
     * @note Ensure thread safety for any shared resources accessed by callbacks
     * in this group.
     */
    rclcpp::CallbackGroup::SharedPtr reentrant_group_;

    /**
     * @brief Callback group that ensures mutually exclusive execution of callbacks.
     *
     * This callback group ensures that only one callback within the group executes
     * at any given time, regardless of the number of available threads in the
     * executor. It is suitable for callbacks that share resources or require
     * serialized execution to avoid race conditions.
     *
     * @note Callbacks in this group will not execute concurrently even with a
     * MultiThreadedExecutor.
     */
    rclcpp::CallbackGroup::SharedPtr mutually_exclusive_group_;
    /**
     * @brief Timer callback which logs a simple message in the terminal.
     *
     */
    void timer1_callback();
    /**
     * @brief Timer callback which logs a simple message in the terminal.
     *
     */
    void timer2_callback();
    /**
     * @brief Subscriber callback for the `chatter` topic
     *
     */
    void chatter_callback(const std_msgs::msg::String::SharedPtr msg);

    /**
     * @brief ROS 2 timer for scheduling periodic terminal logging.
     */
    rclcpp::TimerBase::SharedPtr timer1_;
    /**
     * @brief ROS 2 timer for scheduling periodic terminal logging.
     */
    rclcpp::TimerBase::SharedPtr timer2_;
    /**
     * @brief Subscriber to the `chatter` topic
     *
     */
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
