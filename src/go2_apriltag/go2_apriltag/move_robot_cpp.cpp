#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;

class RotateRobot : public rclcpp::Node {
public:
    RotateRobot() : Node("rotate_robot"), running_time(0.0) {
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&RotateRobot::rotate, this));
    }

private:
    void rotate() {
        if (running_time < 3.0) {
            geometry_msgs::msg::TwistStamped cmd_vel;
            cmd_vel.header.stamp = this->get_clock()->now();
            cmd_vel.twist.angular.z = 0.5;  // Rotate at 0.5 rad/s
            pub_cmd_vel_->publish(cmd_vel);
            running_time += 0.1; // Increment elapsed time
        } else {
            stop_rotation();
        }
    }

    void stop_rotation() {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = this->get_clock()->now();
        cmd_vel.twist.angular.z = 0.0;  // Stop rotation
        pub_cmd_vel_->publish(cmd_vel);
        RCLCPP_INFO(this->get_logger(), "Rotation stopped after 3 seconds.");
        timer_->cancel();
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_vel_;
    rclcpp::TimerBase::SharedPtr timer_;
    double running_time;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RotateRobot>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}