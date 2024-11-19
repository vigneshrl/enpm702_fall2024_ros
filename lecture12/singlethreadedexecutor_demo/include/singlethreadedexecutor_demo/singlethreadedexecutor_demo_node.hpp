#pragma once

#include "color_utils.hpp"
#include "rclcpp/rclcpp.hpp"

class SingleThreadedExecutorDemoNodeOne : public rclcpp::Node
{
public:
    SingleThreadedExecutorDemoNodeOne()
        : Node("node1")
    {

        // Initialize timer1_
        timer1_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&SingleThreadedExecutorDemoNodeOne::timer1, this));
        // Initialize timer2_
        timer2_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&SingleThreadedExecutorDemoNodeOne::timer2, this));

        RCLCPP_INFO_STREAM(this->get_logger(), output_yellow("== node1 started =="));
    }

private:
    /**
     * @brief Timer callback which logs a simple message in the terminal.
     *
     */
    void timer1();
    /**
     * @brief Timer callback which logs a simple message in the terminal.
     *
     */
    void timer2();
    /**
     * @brief ROS 2 timer for scheduling periodic terminal logging.
     */
    rclcpp::TimerBase::SharedPtr timer1_;
    /**
     * @brief ROS 2 timer for scheduling periodic terminal logging.
     */
    rclcpp::TimerBase::SharedPtr timer2_;
};


class SingleThreadedExecutorDemoNodeTwo : public rclcpp::Node
{
public:
    SingleThreadedExecutorDemoNodeTwo()
        : Node("node2")
    {

        // Initialize timer1_
        timer1_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&SingleThreadedExecutorDemoNodeTwo::timer1, this));
        // Initialize timer2_
        timer2_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&SingleThreadedExecutorDemoNodeTwo::timer2, this));

        RCLCPP_INFO_STREAM(this->get_logger(), output_yellow("== node2 started =="));
    }

private:
    /**
     * @brief Timer callback which logs a simple message in the terminal.
     *
     */
    void timer1();
    /**
     * @brief Timer callback which logs a simple message in the terminal.
     *
     */
    void timer2();
    /**
     * @brief ROS 2 timer for scheduling periodic terminal logging.
     */
    rclcpp::TimerBase::SharedPtr timer1_;
    /**
     * @brief ROS 2 timer for scheduling periodic terminal logging.
     */
    rclcpp::TimerBase::SharedPtr timer2_;
};
