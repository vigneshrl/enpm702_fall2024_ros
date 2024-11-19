#include <singlethreadedexecutor_demo_node.hpp>
#include <rclcpp/rclcpp.hpp>

void SingleThreadedExecutorDemoNodeOne::timer1()
{
    RCLCPP_INFO_STREAM(this->get_logger(), output_green("node1 timer1"));
}

void SingleThreadedExecutorDemoNodeOne::timer2()
{
    RCLCPP_INFO_STREAM(this->get_logger(), output_green("node1 timer2"));
}

void SingleThreadedExecutorDemoNodeTwo::timer1()
{
    RCLCPP_INFO_STREAM(this->get_logger(), output_red("node2 timer1"));
}

void SingleThreadedExecutorDemoNodeTwo::timer2()
{
    RCLCPP_INFO_STREAM(this->get_logger(), output_red("node2 timer2"));
}