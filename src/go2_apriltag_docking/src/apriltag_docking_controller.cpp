#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

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
        10ms, std::bind(&AprilTagDockingController::publish_cmd_vel, this));

        // apriltag pose subscriber
        subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10, std::bind(&AprilTagDockingController::apriltag_pose_callback, this, _1));
          
        last_detection_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "AprilTag Docking Controller Initialized!");
    
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;


    // Docking control variables
    bool apriltag_detected_ = false;
    double x_error_ = 0.0;
    double y_error_ = 0.0;
    double theta_error_ = 0.0;

    rclcpp::Time last_detection_time_;
    double detection_timeout_ = 0.5; 

    // Control gains
    double k_x_ = 0.2;
    double k_y_ = 0.3;
    double k_theta_ = 0.3;
    
    // Docking thresholds
    double min_x_threshold_ = 0.25; // Stop when within 25cm
    double min_y_threshold_ = 0.08;
    double min_theta_threshold_ = 0.03;

    void publish_cmd_vel()
    {
      double v_x = 0.0;
      double omega_z = 0.0;
      
      
      const auto elapsed = (this->now() - last_detection_time_).seconds();
      apriltag_detected_ = (elapsed < detection_timeout_);
      
      if (apriltag_detected_){
        // Computing control velocities
        v_x = k_x_ * x_error_;
        omega_z = (k_y_ * y_error_) + (k_theta_ * theta_error_);

        RCLCPP_INFO(this->get_logger(), "Error: %f", x_error_);
        v_x = std::clamp(v_x, 0.0, 0.3);
        omega_z = std::clamp(omega_z, -0.3, 0.3);
        

        // Stop robot when docking is reached
        if (std::abs(x_error_) <= min_x_threshold_ && std::abs(y_error_) <= min_y_threshold_ && std::abs(theta_error_) <= min_theta_threshold_)
        {
          v_x = 0.0;
          omega_z = 0.0;
          RCLCPP_INFO(this->get_logger(), "Docking Complete! Stopping the robot.");
        }


      } else {
        RCLCPP_WARN(this->get_logger(), "AprilTag not detected. Robot will not move.");
          v_x = 0.0;
          omega_z = 0.0;
      }

      auto message = sensor_msgs::msg::Joy();
      message.header.stamp = this->get_clock()->now();
      message.header.frame_id = "teleop_panel";

      message.axes = {0.0, 0.0, 1.0, float(omega_z), float(v_x), 1.0, 0.0, 0.0}; // indices: angular.z: 3, linear.x: 4
      message.buttons = {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0};
      
      
      RCLCPP_INFO(this->get_logger(), "Publishing Joy message \n Forward: %f, Rotation: %f", v_x, omega_z);
      publisher_->publish(message);


    }

    void apriltag_pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        // apriltag_detected_ = false;
        for (const auto &transform : msg->transforms)
        {
            if (transform.child_frame_id == "elevator_docking")
            {
              last_detection_time_ = this->now();
                // apriltag_detected_ = true;

                x_error_ = transform.transform.translation.z; // Forward distance
                y_error_ = transform.transform.translation.x;
                // Convert quaternion to Euler angles
                tf2::Quaternion q(
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w);
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                theta_error_ = pitch; // Orientation misalignment y direction

                //RCLCPP_INFO(this->get_logger(),
                 //           "AprilTag Pose -> x: %.2f",
                  //          x_error_);
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

// some additional comment from the branch of branch of branch