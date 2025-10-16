#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>

// For real robot
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using namespace std::chrono_literals;

class CmdVelRelay : public rclcpp::Node
{
public:
    CmdVelRelay()
        : Node("cmd_vel_relay")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        go2_request_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel_robot", 10,
            std::bind(&CmdVelRelay::cmdVelCallback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Cmd Vel Relay Node Started - Listening on /cmd_vel_robot");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr go2_request_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;

    void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        // Forward the received command to /cmd_vel topic
        // cmd_vel_pub_->publish(*msg);  // Commented out for debugging real robot movement
        
        // Send to Unitree API
        unitree_api::msg::Request req;
        SportClient sport_req;
        
        // Check if it's a stop command (all velocities are zero)
        if (msg->twist.linear.x == 0.0 && msg->twist.linear.y == 0.0 && msg->twist.angular.z == 0.0)
        {
            sport_req.StopMove(req);
            RCLCPP_INFO(this->get_logger(), "Stopping Robot...");
        }
        else
        {
            sport_req.Move(req, msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z);
            RCLCPP_INFO(this->get_logger(), "Relaying cmd_vel: linear=[%.2f, %.2f, %.2f], angular=[%.2f, %.2f, %.2f]",
                       msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
                       msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);
        }
        
        go2_request_pub_->publish(req);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelRelay>());
    rclcpp::shutdown();
    return 0;
}
