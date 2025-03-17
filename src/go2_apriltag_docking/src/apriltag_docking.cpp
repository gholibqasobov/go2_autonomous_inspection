/*
steps to solve this problem

subscribe to /tf of apriltag
publish to /joy of autonomy_go2

algorithm
goal pose
approach by turning

it's better to start from joy publisher -- done
apriltag_subscriber -- done (in a separate script)

*/ 

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class DockingCmdVelPublisher : public rclcpp::Node
{
  public:
    DockingCmdVelPublisher()
    : Node("docking_cmd_vel_publisher")
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&DockingCmdVelPublisher::publish_cmd_vel, this));
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_cmd_vel()
    {
        auto message = sensor_msgs::msg::Joy();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "teleop_panel";


        message.axes = {0.0, 0.0, 1.0, 0.0, 0.5, 1.0, 0.0, 0.0}; // indices angular.z: 3, linear.x: 4
        message.buttons = {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0};
        
        
        RCLCPP_INFO(this->get_logger(), "Publishing Joy message \n Forward: %f, Rotation: %f", message.axes[3], message.axes[4]);
        publisher_->publish(message);


    }
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockingCmdVelPublisher>());
  rclcpp::shutdown();
  return 0;
}

// adding a /tf apriltag_subscriber