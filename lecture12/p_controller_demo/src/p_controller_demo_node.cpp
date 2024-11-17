#include "p_controller_demo_node.hpp"

void ProportionalController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract current position
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    // Extract current orientation as yaw (theta)
    tf2::Quaternion quaternion;
    tf2::fromMsg(msg->pose.pose.orientation, quaternion);
    tf2::Matrix3x3(quaternion).getRPY(roll_, pitch_, current_theta_);

    control_loop();
}

void ProportionalController::control_loop() {
    // Calculate distance and angular errors
    double distance_error = std::sqrt(std::pow(goal_x_ - current_x_, 2) + std::pow(goal_y_ - current_y_, 2));
    double desired_angle = std::atan2(goal_y_ - current_y_, goal_x_ - current_x_);
    double angle_error = normalize_angle(desired_angle - current_theta_);
    double final_orientation_error = normalize_angle(goal_theta_ - current_theta_);

    // Phase 1: Navigate to goal position
    if (distance_error > epsilon_) {
        // Compute control signals
        double linear_velocity = kp_distance_ * distance_error;
        double angular_velocity = kp_angle_ * angle_error;

        // Publish velocity commands
        publish_velocity(linear_velocity, angular_velocity);
    }
    // Phase 2: Adjust final orientation
    else if (std::abs(final_orientation_error) > epsilon_theta_) {
        // Stop linear movement and adjust angular velocity
        double angular_velocity = kp_angle_ * final_orientation_error;
        publish_velocity(0.0, angular_velocity);
    }
    // Goal reached with correct orientation
    else {
        publish_velocity(0.0, 0.0);
        RCLCPP_INFO(this->get_logger(), "Goal reached with correct orientation!");
    }
}

void ProportionalController::publish_velocity(double linear, double angular) {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear;
    msg.angular.z = angular;
    velocity_publisher_->publish(msg);
}

double ProportionalController::normalize_angle(double angle) {
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}