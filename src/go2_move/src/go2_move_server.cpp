#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"  // Include Joy message header
#include "geometry_msgs/msg/pose_stamped.hpp"

// For real robot
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using namespace std::chrono_literals;

class MoveRobotServer : public rclcpp::Node
{
public:
    MoveRobotServer()
        : Node("move_robot_server")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        go2_request_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);  // Add publisher for /joy
        odom_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/utlidar/robot_pose",
            10,
            std::bind(&MoveRobotServer::robotPoseCallback, this, std::placeholders::_1));


        goal_x_ = 0.0;
        goal_y_ = 0.0;
        goal_theta_ = 0.0;

        current_x_ = 0.0;
        current_y_ = 0.0;
        current_theta_ = 0.0;

        init_x_ = 0.0;
        init_y_ = 0.0;
        init_theta_ = 0.0;
        init_pose_received_ = false;

        position_tolerance_ = 0.05;
        angle_tolerance_ = 0.05;

        vel_ = 0.3;

        is_moving_ = false;

        timer_ = this->create_wall_timer(100ms, std::bind(&MoveRobotServer::controlLoop, this));
    }

    void moveToGoal(double x, double y, double theta)
    {
        goal_x_ = x;
        goal_y_ = y;
        goal_theta_ = theta;

        RCLCPP_INFO(this->get_logger(), "Setting new goal: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);
        
        is_moving_ = true;

    }

private:
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr go2_request_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;  // Define Joy publisher
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;

    // Goal position
    double goal_x_;
    double goal_y_;
    double goal_theta_;
    
    // Current position
    double current_x_;
    double current_y_;
    double current_theta_;
    
    // Control parameters
    double position_tolerance_;
    double angle_tolerance_;
    
    // Movement state
    bool is_moving_;

    double init_x_;
    double init_y_;
    double init_theta_;
    bool init_pose_received_;

    double vel_;

    


    void robotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        // quaternion to Euler

        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        current_theta_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        


        current_theta_ = msg->pose.pose.orientation.z;

        RCLCPP_DEBUG(this->get_logger(), "Current pose: x=%.2f, y=%.2f, theta=%.2f", 
                    current_x_, current_y_, current_theta_);
        
        if(!init_pose_received_)
        {
            init_x_ = current_x_;
            init_y_ = current_y_;
            init_theta_ = current_theta_;
            init_pose_received_ = true;
        }
    }


    void controlLoop()
    {
        if(!is_moving_ || !init_pose_received_) 
        {
            return;
        }
        
        // Error in each direction 
        double x_error = (init_x_ + goal_x_) - current_x_;
        double y_error = (init_y_ + goal_y_) - current_y_;
        double theta_error = (init_theta_ + goal_theta_) - current_theta_;
        
        // Check if reached the goal position within tolerance
        bool reached_x = std::abs(x_error) < position_tolerance_;
        bool reached_y = std::abs(y_error) < position_tolerance_;
        bool reached_theta = std::abs(theta_error) < angle_tolerance_;

        if (reached_x && reached_y && reached_theta) {
            RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping robot.");
            stopRobot();
            is_moving_ = false;
            return;
        }


        // Calculate velocity commands
        double x_vel = 0.0;
        double y_vel = 0.0;
        double theta_vel = 0.0;

        if (!reached_x) {
            x_vel = vel_ * (x_error > 0 ? 1.0 : -1.0);
        }
        
        if (!reached_y) {
            y_vel = vel_ * (y_error > 0 ? 1.0 : -1.0);
        }
        
        if (!reached_theta) {
            theta_vel = vel_ * (theta_error > 0 ? 1.0 : -1.0);  // Slower rotation
        }

        RCLCPP_INFO(this->get_logger(), "Moving: x_vel=%.2f, y_vel=%.2f, theta_vel=%.2f", 
                   x_vel, y_vel, theta_vel);
        
        publishVelocityCommands(x_vel, y_vel, theta_vel);


    }

    

    void publishVelocityCommands(float x_vel=0.0, float y_vel=0.0, float theta_vel=0.0)
    {

        // Publish TwistStamped message
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = this->now();
        cmd_vel.header.frame_id = "vehicle";
        cmd_vel.twist.linear.x = x_vel;
        cmd_vel.twist.linear.y = y_vel;
        cmd_vel.twist.angular.z = theta_vel;

        RCLCPP_INFO(this->get_logger(), "Moving Forward...");
        cmd_vel_pub_->publish(cmd_vel);

        // Publish Joy message
        sensor_msgs::msg::Joy joy_msg;
        joy_msg.header.stamp = this->now();
        joy_msg.header.frame_id = "teleop_panel";

        // Example joystick data
        joy_msg.axes = {theta_vel, 0.0, 0.0, y_vel, x_vel, -1.0, 0.0, 0.0}; // index 0: rotat.z, index 3: linear.y, index 4: linear.x, index 5: manual_mode;
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
    // rclcpp::spin(std::make_shared<MoveRobotServer>());
    auto node = std::make_shared<MoveRobotServer>();

    node->moveToGoal(0.0, 1.0, 0.0);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
