#include <processing_node.hpp>

void ProcessingDemo::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received LiDAR data: ranges[0]=%.2f", msg->ranges[0]);
}

void ProcessingDemo::camera_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received Camera data: width=%u, height=%u", msg->width, msg->height);
}

void ProcessingDemo::temperature_callback(const sensor_msgs::msg::Temperature::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received Temperature data: %.2f°C", msg->temperature);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ProcessingDemo>();

    // Create a MultiThreadedExecutor
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add more nodes to the executor if needed
    executor.add_node(node);

    // Spin the executor
    executor.spin();
    rclcpp::shutdown();
}
