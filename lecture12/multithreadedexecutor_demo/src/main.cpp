#include <multithreadedexecutor_demo_node.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiThreadedExecutorDemoNode>();

    // Create a SingleThreadedExecutor
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add more nodes to the executor if needed
    executor.add_node(node);

    // Spin the executor
    executor.spin();
    rclcpp::shutdown();
}