/*
steps to solve this problem

subscribe to /tf of apriltag
publish to /joy of autonomy_go2

algorithm
goal pose
approach by turning

it's better to start from joy publisher -- done
apriltag_subscriber -- done (in a separate script)

merge two pub and sub into one script -- 

*/ 

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class AprilTagDockingController : public rclcpp::Node
{
  public:
    AprilTagDockingController()
    : Node("apriltag_docking_controller")
    {
        // angular and linear velocity publisher to joy topic
        publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);
        timer_ = this->create_wall_timer(
        100ms, std::bind(&AprilTagDockingController::publish_cmd_vel, this));

        // apriltag pose subscriber
        subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10, std::bind(&AprilTagDockingController::apriltag_pose_callback, this, _1));
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
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

    void apriltag_pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto &transform : msg->transforms)
        {
            if (transform.child_frame_id == "elevator_docking")
            {
                RCLCPP_INFO(this->get_logger(), "Received transform for frame: %s", transform.child_frame_id.c_str());
                RCLCPP_INFO(this->get_logger(), "Translation: x = %f, y = %f, z = %f",
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z);
                RCLCPP_INFO(this->get_logger(), "Rotation (Quaternion): x = %f, y = %f, z = %f, w = %f",
                            transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z,
                            transform.transform.rotation.w);
            }
        }
    }
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagDockingController>());
  rclcpp::shutdown();
  return 0;
}
