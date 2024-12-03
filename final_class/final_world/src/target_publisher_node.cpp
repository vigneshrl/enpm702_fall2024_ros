
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <target_publisher_node.hpp>
#include <rclcpp/rclcpp.hpp>

void TargetPublisher::publish_target() {
    target_msg_.header.stamp = this->now();
    target_msg_.header.frame_id = "box_frame";

    // Populate position
    target_msg_.pose.position.x = 4.0;
    target_msg_.pose.position.y = 4.0;
    target_msg_.pose.position.z = 0.0;

    // Populate orientation (as a quaternion)
    target_msg_.pose.orientation.x = 0.0;
    target_msg_.pose.orientation.y = 0.0;
    target_msg_.pose.orientation.z = 0.0;
    target_msg_.pose.orientation.w = 1.0;

    // Publish the message
    // RCLCPP_INFO(this->get_logger(), "Publishing PoseStamped message");
    target_publisher_->publish(target_msg_);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetPublisher>("target_publisher"));
    rclcpp::shutdown();
}