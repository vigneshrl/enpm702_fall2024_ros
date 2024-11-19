#include <singlethreadedexecutor_demo_node.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node1 = std::make_shared<SingleThreadedExecutorDemoNodeOne>();
    auto node2 = std::make_shared<SingleThreadedExecutorDemoNodeTwo>();

    // Create a SingleThreadedExecutor
    rclcpp::executors::SingleThreadedExecutor executor;

    // Add both nodes to the executor
    executor.add_node(node1);
    executor.add_node(node2);

    // Spin the executor
    executor.spin();
    rclcpp::shutdown();
}