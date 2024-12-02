#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class TargetPublisher : public rclcpp::Node {
   public:
    TargetPublisher(std::string node_name)
        : rclcpp::Node(node_name) {
        // Initialize the publisher object
        target_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target", 10);
        // Initialize the message to be published
        target_msg_ = geometry_msgs::msg::PoseStamped();
        // Initialize the timer
        pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&TargetPublisher::publish_target, this));
    }

   private:
    void publish_target();
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_publisher_;
    geometry_msgs::msg::PoseStamped target_msg_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
};