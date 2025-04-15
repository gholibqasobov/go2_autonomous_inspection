#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// For real robot
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using namespace std::chrono_literals;

class AprilTagDockingController : public rclcpp::Node
{
public:
  AprilTagDockingController()
  : Node("simple_apriltag_docking_node")
  {
    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    go2_request_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);
    
    // Subscribers
    apriltag_tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", 10, 
        std::bind(&AprilTagDockingController::tfCallback, this, std::placeholders::_1));
    
    // Control loop timer
    timer_ = this->create_wall_timer(
        100ms, std::bind(&AprilTagDockingController::controlLoop, this));
    
    // Parameters
    this->declare_parameter("tag_frame_id", std::string("elevator_docking"));
    this->declare_parameter("k_rotation", 0.5);
    this->declare_parameter("k_forward", 0.3);      // Forward motion gain
    this->declare_parameter("max_forward_speed", 0.3);
    this->declare_parameter("max_angular_speed", 0.5);
    this->declare_parameter("center_threshold", 0.05);  // Threshold for considering tag centered
    this->declare_parameter("final_threshold", 0.3);   // Final docking distance
    this->declare_parameter("final_yaw_threshold", 0.05); // Final yaw alignment threshold (radians)
    this->declare_parameter("tag_timeout", 1.0);        // Seconds to consider tag lost
    
    tag_frame_id_ = this->get_parameter("tag_frame_id").as_string();
    k_rotation_ = this->get_parameter("k_rotation").as_double();
    k_forward_ = this->get_parameter("k_forward").as_double();
    max_forward_speed_ = this->get_parameter("max_forward_speed").as_double();
    max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
    center_threshold_ = this->get_parameter("center_threshold").as_double();
    final_threshold_ = this->get_parameter("final_threshold").as_double();
    final_yaw_threshold_ = this->get_parameter("final_yaw_threshold").as_double();
    tag_timeout_ = this->get_parameter("tag_timeout").as_double();

    last_detection_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Simple AprilTag Docking Node started");
  }

private:
  // Publishers and subscribers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr go2_request_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr apriltag_tf_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Parameters
  std::string tag_frame_id_;
  double k_rotation_;
  double k_forward_;        // Forward motion gain
  double max_forward_speed_;
  double max_angular_speed_;
  double center_threshold_;
  double final_threshold_;
  double final_yaw_threshold_;
  double tag_timeout_;
  
  // State variables
  bool tag_detected_ = false;
  double x_error_ = 0.0;     // Distance to tag in z (forward in camera frame)
  double y_error_ = 0.0;     // Distance to tag in x (lateral in camera frame)
  double pitch_error_ = 0.0;   // Angular error
  rclcpp::Time last_detection_time_;
  bool docking_complete_ = false;
  
  // State machine states
  enum DockingState {
    SEARCHING,
    ROTATING,
    APPROACHING,
    FINAL_ALIGNMENT,
    DOCKED
  };
  DockingState current_state_ = SEARCHING;
  
  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (const auto &transform : msg->transforms)
    {
      if (transform.child_frame_id == tag_frame_id_)
      {
        // Get pose relative to camera
        last_detection_time_ = this->now();
        tag_detected_ = true;

        // Camera frame: z is forward, x is right
        x_error_ = transform.transform.translation.z;  // Forward distance
        y_error_ = -transform.transform.translation.x;  // Lateral offset
        
        // Convert quaternion to Euler angles
        tf2::Quaternion q(
          transform.transform.rotation.x,
          transform.transform.rotation.y,
          transform.transform.rotation.z,
          transform.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        pitch_error_ = pitch;
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Tag detected - Forward: %.2f m, Lateral: %.2f m, Pitch: %.2f rad", 
                           x_error_, y_error_, pitch_error_);
        return;
      }
    }
    
    // Check for tag timeout
    if (tag_detected_ && (this->now() - last_detection_time_).seconds() > tag_timeout_)
    {
      RCLCPP_WARN(this->get_logger(), "Tag lost - timeout reached");
      tag_detected_ = false;
      current_state_ = SEARCHING;
    }
  }
  
  void controlLoop()
  {
    if (docking_complete_)
    {
      return;
    }
    
    if (!tag_detected_ || (this->now() - last_detection_time_).seconds() > tag_timeout_)
    {
      // If no tag is detected or detection timeout, stop the robot and search
      stopRobot();
      current_state_ = SEARCHING;
      RCLCPP_INFO(this->get_logger(), "Searching for AprilTag... [lin_vel: 0.00, rot_vel: 0.00]");
      return;
    }
    
    // Implement the simple state machine for docking
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = this->now();
    cmd_vel.header.frame_id = "vehicle";
    
    switch (current_state_)
    {
      case SEARCHING:
        if (tag_detected_)
        {
          current_state_ = ROTATING;
          RCLCPP_INFO(this->get_logger(), "Tag found. Rotating to center... [lin_vel: 0.00, rot_vel: 0.00]");
        }
        break;
        
      case ROTATING:
        // Rotate to center the tag
        if (std::abs(y_error_) < center_threshold_)
        {
          // Tag is centered, stop rotation and start approaching
          current_state_ = APPROACHING;
          cmd_vel.twist.angular.z = 0.0;
          RCLCPP_INFO(this->get_logger(), "Tag centered. Starting approach... [lin_vel: 0.00, rot_vel: 0.00]");
        }
        else
        {
          // Continue rotating
          cmd_vel.twist.angular.z = k_rotation_ * y_error_;
          
          // Limit the rotation speed
          if (cmd_vel.twist.angular.z > max_angular_speed_)
            cmd_vel.twist.angular.z = max_angular_speed_;
          else if (cmd_vel.twist.angular.z < -max_angular_speed_)
            cmd_vel.twist.angular.z = -max_angular_speed_;
            
          RCLCPP_INFO(this->get_logger(), 
                      "ROTATING: offset: %.2f [lin_vel: %.2f, rot_vel: %.2f]", 
                      y_error_, cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
        }
        break;
        
      case APPROACHING:
        // Check if we need to recenter
        if (std::abs(y_error_) > center_threshold_ * 2.0)  // Wider threshold during approach
        {
          // Need to recenter
          current_state_ = ROTATING;
          RCLCPP_INFO(this->get_logger(), 
                      "Tag off-center. Recentering... [lin_vel: %.2f, rot_vel: %.2f]", 
                      cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
          break;
        }
        
        // Check if we've reached the final threshold
        if (x_error_ <= final_threshold_)
        {
          // Reached the docking distance - now perform final alignment
          current_state_ = FINAL_ALIGNMENT;
          RCLCPP_INFO(this->get_logger(), 
                      "Reached docking distance. Performing final alignment... [lin_vel: %.2f, rot_vel: %.2f]", 
                      cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
          break;
        }
        
        // Continue approaching - using proportional control for forward movement
        cmd_vel.twist.linear.x = max_forward_speed_;
        
        // Limit the forward speed
        if (cmd_vel.twist.linear.x > max_forward_speed_)
          cmd_vel.twist.linear.x = max_forward_speed_;
          
        // Slow down as we get closer
        if (x_error_ < final_threshold_ * 3.0)
        {
          cmd_vel.twist.linear.x = std::min(cmd_vel.twist.linear.x, max_forward_speed_ * k_forward_);
        }
        
        // Maintain heading while moving
        cmd_vel.twist.angular.z = k_rotation_ * y_error_ * 0.5; // Reduced gain during approach
        
        RCLCPP_INFO(this->get_logger(), 
                    "APPROACHING: distance: %.2f [lin_vel: %.2f, rot_vel: %.2f]", 
                    x_error_, cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
        break;
        
      case FINAL_ALIGNMENT:
        // Final alignment - correct any remaining yaw error
        if (std::abs(pitch_error_) < final_yaw_threshold_ && std::abs(y_error_) < center_threshold_)
        {
          // Alignment complete, we're fully docked
          stopRobot();
          current_state_ = DOCKED;
          docking_complete_ = true;
          RCLCPP_INFO(this->get_logger(), "Final alignment complete. Docking successful! [lin_vel: 0.00, rot_vel: 0.00]");
        }
        else
        {
          // Perform final alignment
          cmd_vel.twist.angular.z = k_rotation_ * y_error_;
          
          // Limit angular velocity for precision
          if (cmd_vel.twist.angular.z > max_angular_speed_)
            cmd_vel.twist.angular.z = max_angular_speed_;
          else if (cmd_vel.twist.angular.z < -max_angular_speed_)
            cmd_vel.twist.angular.z = -max_angular_speed_;
            
          RCLCPP_INFO(this->get_logger(),
                      "FINAL_ALIGNMENT: lateral: %.2f, yaw: %.2f [lin_vel: %.2f, rot_vel: %.2f]", 
                      y_error_, pitch_error_, cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
        }
        break;
        
      case DOCKED:
        // Do nothing, we're docked
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "DOCKED: Successfully docked to target [lin_vel: 0.00, rot_vel: 0.00]");
        break;
    }
    
    // Send commands
    cmd_vel_pub_->publish(cmd_vel);
    publishJoy(cmd_vel);
    publishUnitreeCommand(cmd_vel);
  }
  
  void publishJoy(const geometry_msgs::msg::TwistStamped& cmd_vel)
  {
    // Convert cmd_vel to Joy message
    sensor_msgs::msg::Joy joy_msg;
    joy_msg.header.stamp = this->now();
    joy_msg.header.frame_id = "teleop_panel";
    
    // Initialize with zeros
    joy_msg.axes.resize(8, 0.0);
    joy_msg.buttons.resize(11, 0);
    
    // Map cmd_vel to joy axes
    // Based on your code:
    // index 0: rotate.z, index 3: linear.y, index 4: linear.x, index 5: manual_mode
    float norm_angular = 0.0;
    if (max_angular_speed_ > 0) {
      norm_angular = cmd_vel.twist.angular.z / max_angular_speed_;  // Normalize to [-0.3, 0.3]
      if (norm_angular > 0.3) norm_angular = 0.3;
      if (norm_angular < -0.3) norm_angular = -0.3;
    }
    
    float norm_linear = 0.0;
    if (max_forward_speed_ > 0) {
      norm_linear = cmd_vel.twist.linear.x / max_forward_speed_;  // Normalize to [-0.3, 0.3]
      if (norm_linear > 0.3) norm_linear = 0.3;
      if (norm_linear < -0.3) norm_linear = -0.3;
    }
    
    joy_msg.axes[0] = norm_angular;
    joy_msg.axes[3] = 0.0;  // No lateral movement in this approach
    joy_msg.axes[4] = norm_linear;
    joy_msg.axes[5] = -1.0;  // Enable manual mode (-1 according to Dosymzhan code)
    
    // Enable manual control button
    joy_msg.buttons[7] = 1;
    
    joy_pub_->publish(joy_msg);
    
    // Log state and commands periodically
    RCLCPP_DEBUG(this->get_logger(), 
                "State: %d, Linear X: %.2f, Angular Z: %.2f", 
                static_cast<int>(current_state_), cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
  }
  
  void publishUnitreeCommand(const geometry_msgs::msg::TwistStamped& cmd_vel)
  {
    unitree_api::msg::Request req;
    SportClient sport_req;
    
    if (cmd_vel.twist.linear.x == 0.0 && 
        cmd_vel.twist.linear.y == 0.0 && 
        cmd_vel.twist.angular.z == 0.0)
    {
      sport_req.StopMove(req);
    }
    else
    {
      sport_req.Move(req, 
                    cmd_vel.twist.linear.x, 
                    cmd_vel.twist.linear.y, 
                    cmd_vel.twist.angular.z);
    }
    
    go2_request_pub_->publish(req);
  }
  
  void stopRobot()
  {
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = this->now();
    cmd_vel.header.frame_id = "vehicle";
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    
    cmd_vel_pub_->publish(cmd_vel);
    
    // Send Joy message with zeroed values
    sensor_msgs::msg::Joy joy_msg;
    joy_msg.header.stamp = this->now();
    joy_msg.header.frame_id = "teleop_panel";
    joy_msg.axes.resize(8, 0.0);
    joy_msg.buttons.resize(11, 0);
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
  auto node = std::make_shared<AprilTagDockingController>();
  
  RCLCPP_INFO(node->get_logger(), "Starting AprilTag docking node with rotate-then-move approach");
  RCLCPP_INFO(node->get_logger(), "Listening for AprilTag TF transforms with frame ID: elevator_docking");
  RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}