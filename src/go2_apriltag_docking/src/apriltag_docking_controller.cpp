/*
steps to solve this problem

subscribe to /tf of apriltag
publish to /joy of autonomy_go2

algorithm
goal pose
approach by turning

it's better to start from joy publisher -- done
apriltag_subscriber -- done (in a separate script)

merge two pub and sub into one script -- done

next
add linear docking controller
add angular docking controller

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
          
        
        RCLCPP_INFO(this->get_logger(), "AprilTag Docking Controller Initialized!");
    
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;


    // Docking control variables
    
    // double y_error_ = 0.0;
    // double theta_error_ = 0.0;
    
    bool apriltag_detected_ = false;
    double x_error_ = 0.0;
    double y_error_ = 0.0;
    double theta_error_ = 0.0;


    // Control gains
    double k_x_ = 0.5;
    double k_y_ = 0.5;
    double k_theta_ = 0.5;
    

    // Docking thresholds
    double min_x_threshold_ = 0.25; // Stop when within 20cm
    double min_y_threshold_ = 0.03;
    double min_theta_threshold_ = 0.05;





    void publish_cmd_vel()
    {
      // Computing control velocities
      double v_x = k_x_ * x_error_;
      double omega_z = (k_y_ * y_error_) + (k_theta_ * theta_error_);


      RCLCPP_INFO(this->get_logger(), "Error: %f", x_error_);
      v_x = std::clamp(v_x, -0.5, 0.5);
      omega_z = std::clamp(omega_z, -1.0, 1.0);

      if (!apriltag_detected_)
      {
          RCLCPP_WARN(this->get_logger(), "AprilTag not detected. Robot will not move.");
          v_x = 0.0;
          omega_z = 0.0;
          return;
      }
        auto message = sensor_msgs::msg::Joy();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "teleop_panel";


      


      // Stop robot when docking is reached
      if (std::abs(x_error_) < min_x_threshold_ && std::abs(y_error_) < min_y_threshold_ && std::abs(theta_error_) < min_theta_threshold_)
      {
        v_x = 0.0;
        omega_z = 0.0;
        RCLCPP_INFO(this->get_logger(), "Docking Complete! Stopping the robot.");
      }






        message.axes = {0.0, 0.0, 1.0, float(omega_z), float(v_x), 1.0, 0.0, 0.0}; // indices: angular.z: 3, linear.x: 4
        message.buttons = {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0};
        
        
        RCLCPP_INFO(this->get_logger(), "Publishing Joy message \n Forward: %f, Rotation: %f", message.axes[3], message.axes[4]);
        publisher_->publish(message);


    }

    void apriltag_pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        apriltag_detected_ = false;
        for (const auto &transform : msg->transforms)
        {
            if (transform.child_frame_id == "elevator_docking")
            {
                apriltag_detected_ = true;

                x_error_ = transform.transform.translation.z; // Forward distance


                // RCLCPP_INFO(this->get_logger(), "Received transform for frame: %s", transform.child_frame_id.c_str());
                // RCLCPP_INFO(this->get_logger(), "Translation: x = %f, y = %f, z = %f",
                //             transform.transform.translation.x,
                //             transform.transform.translation.y,
                //             transform.transform.translation.z);
                // RCLCPP_INFO(this->get_logger(), "Rotation (Quaternion): x = %f, y = %f, z = %f, w = %f",
                //             transform.transform.rotation.x,
                //             transform.transform.rotation.y,
                //             transform.transform.rotation.z,
                //             transform.transform.rotation.w);

                RCLCPP_INFO(this->get_logger(),
                            "AprilTag Pose -> x: %.2f",
                            x_error_);
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

// some additional comment