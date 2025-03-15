#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

// For real robot
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"




class DockingCmdVelPublisher : public rclcpp::Node
{
  public:
    DockingCmdVelPublisher()
    : Node("docking_cmd_vel_publisher")
    {
    //   publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        autonomy_mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/autonomy_mode", 10);
        go2_request_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        moveForward();
        stopRobot();
    
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr go2_request_pub_;



    void moveForward() {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = this->now();
        cmd_vel.header.frame_id = "vehicle";

        cmd_vel.twist.linear.x = 0.2; 
        cmd_vel.twist.linear.y = 0.0;
        cmd_vel.twist.angular.z = 0.0;

        RCLCPP_INFO(this->get_logger(), "Moving Forward for 3 seconds...");
        cmd_vel_pub_->publish(cmd_vel);

        
        // If using Unitree API, send direct movement request
        unitree_api::msg::Request req;
        SportClient sport_req;
        sport_req.Move(req, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);
        go2_request_pub_->publish(req);

        std::this_thread::sleep_for(std::chrono::seconds(3)); // Move for 3 seconds
    }

    void stopRobot() {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = this->now();
        cmd_vel.header.frame_id = "vehicle";

        // Stop the robot
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.linear.y = 0.0;
        cmd_vel.twist.angular.z = 0.0;

        RCLCPP_INFO(this->get_logger(), "Stopping Robot...");
        cmd_vel_pub_->publish(cmd_vel);

        // If using Unitree API, stop robot
        unitree_api::msg::Request req;
        SportClient sport_req;
        sport_req.StopMove(req);
        go2_request_pub_->publish(req);
    }

};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::spin(std::make_shared<DockingCmdVelPublisher>());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}