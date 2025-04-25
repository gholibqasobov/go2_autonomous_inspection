#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"  // Include Joy message header

// For real robot
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using namespace std::chrono_literals;

class MoveRobotServer : public rclcpp::Node
{
public:
    MoveRobotServer()
        : Node("docking_cmd_vel_publisher"), move_duration_(3s)
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        go2_request_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);  // Add publisher for /joy

        start_time_ = this->now();
        timer_ = this->create_wall_timer(100ms, std::bind(&MoveRobotServer::moveForward, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr go2_request_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;  // Define Joy publisher

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
    std::chrono::seconds move_duration_;

    void moveForward()
    {
        if ((this->now() - start_time_).seconds() >= move_duration_.count())
        {
            stopRobot();
            return;
        }

        // Publish TwistStamped message
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = this->now();
        cmd_vel.header.frame_id = "vehicle";
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.linear.y = 0.0;
        cmd_vel.twist.angular.z = 0.0;

        RCLCPP_INFO(this->get_logger(), "Moving Forward...");
        cmd_vel_pub_->publish(cmd_vel);

        // Publish Joy message
        sensor_msgs::msg::Joy joy_msg;
        joy_msg.header.stamp = this->now();
        joy_msg.header.frame_id = "teleop_panel";

        // Example joystick data
        joy_msg.axes = {1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0}; // index 0: rotat.z, index 3: linear.y, index 4: linear.x, index 5: manual_mode;
        joy_msg.buttons = {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0};  

        joy_pub_->publish(joy_msg);  // Publish to /joy

        // If using Unitree API, send movement request
        unitree_api::msg::Request req;
        SportClient sport_req;
        sport_req.Move(req, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);
        go2_request_pub_->publish(req);
    }

    void stopRobot()
    {
        RCLCPP_INFO(this->get_logger(), "Stopping Robot...");
        timer_->cancel();

        // Stop movement
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = this->now();
        cmd_vel.header.frame_id = "vehicle";
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.linear.y = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);

        // Send Joy message with zeroed values (to indicate stopping)
        sensor_msgs::msg::Joy joy_msg;
        joy_msg.header.stamp = this->now();
        joy_msg.header.frame_id = "teleop_panel";
        joy_msg.axes = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  
        joy_msg.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  
        joy_pub_->publish(joy_msg);

        // Stop Unitree movement
        unitree_api::msg::Request req;
        SportClient sport_req;
        sport_req.StopMove(req);
        go2_request_pub_->publish(req);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveRobotServer>());
    rclcpp::shutdown();
    return 0;
}
