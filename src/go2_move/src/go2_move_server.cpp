#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

// Action
#include "rclcpp_action/rclcpp_action.hpp"
#include "go2_interfaces/action/move_to_goal.hpp"

using namespace std::chrono_literals;
using MoveToGoal = go2_interfaces::action::MoveToGoal;
using GoalHandleMoveToGoal = rclcpp_action::ServerGoalHandle<MoveToGoal>;

class MoveRobotServer : public rclcpp::Node
{
public:
    MoveRobotServer()
        : Node("move_robot_server")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        go2_request_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);

        this->declare_parameter("x_vel", 0.3);
        this->declare_parameter("y_vel", 0.3);
        this->declare_parameter("theta_vel", 0.3);
        this->declare_parameter("disp_x", 0.35);
        this->declare_parameter("disp_y", 0.17);
        this->declare_parameter("disp_theta", 0.45);

        x_vel_ = this->get_parameter("x_vel").as_double();
        y_vel_ = this->get_parameter("y_vel").as_double();
        theta_vel_ = this->get_parameter("theta_vel").as_double();
        disp_x_ = this->get_parameter("disp_x").as_double();
        disp_y_ = this->get_parameter("disp_y").as_double();
        disp_theta_ = this->get_parameter("disp_theta").as_double();

        // Action Server
        action_server_ = rclcpp_action::create_server<MoveToGoal>(
            this,
            "move_to_goal",
            std::bind(&MoveRobotServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveRobotServer::handleCancel, this, std::placeholders::_1),
            std::bind(&MoveRobotServer::handleAccepted, this, std::placeholders::_1)
        );
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr go2_request_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;

    rclcpp_action::Server<MoveToGoal>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_;

    double x_vel_, y_vel_, theta_vel_;
    double disp_x_, disp_y_, disp_theta_;

    double x_duration_, y_duration_, theta_duration_;
    double x_sign_, y_sign_, theta_sign_;
    
    std::shared_ptr<GoalHandleMoveToGoal> active_goal_handle_;

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveToGoal::Goal> goal)
    {
        // Fix format specifiers to match the actual types
        RCLCPP_INFO(this->get_logger(), "Received goal request: x=%.2f, y=%.2f, theta=%.2f", goal->x, goal->y, goal->theta);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleMoveToGoal> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleMoveToGoal> goal_handle)
    {
        // Start executing the goal
        active_goal_handle_ = goal_handle;
        auto goal = goal_handle->get_goal();

        x_duration_ = std::abs(goal->x);
        y_duration_ = std::abs(goal->y);
        theta_duration_ = std::abs(goal->theta);

        x_sign_ = (goal->x < 0) ? -1.0 : 1.0;
        y_sign_ = (goal->y < 0) ? -1.0 : 1.0;
        theta_sign_ = (goal->theta) ? -1.0 : 1.0;


        start_time_ = this->now();

        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&MoveRobotServer::moveRobot, this)
        );
    }

    void moveRobot()
    {
        if (!active_goal_handle_) {
            return;
        }

        auto now = this->now();
        double elapsed = (now - start_time_).seconds();

        float vx = (elapsed < x_duration_/disp_x_) ? x_vel_ * x_sign_ : 0.0;
        float vy = (elapsed < y_duration_/disp_y_) ? y_vel_ * y_sign_ : 0.0;
        float vtheta = (elapsed < theta_duration_/disp_theta_) ? theta_vel_ * theta_sign_ : 0.0;

        // Send feedback
        auto feedback = std::make_shared<MoveToGoal::Feedback>();
        feedback->current_state = "Moving...";
        active_goal_handle_->publish_feedback(feedback);

        // Publish velocity commands
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = now;
        cmd_vel.header.frame_id = "vehicle";
        cmd_vel.twist.linear.x = vx;
        cmd_vel.twist.linear.y = vy;
        cmd_vel.twist.angular.z = vtheta;
        cmd_vel_pub_->publish(cmd_vel);

        // Publish Joy
        sensor_msgs::msg::Joy joy_msg;
        joy_msg.header.stamp = now;
        joy_msg.header.frame_id = "teleop_panel";
        joy_msg.axes = {vtheta, 0.0, 0.0, vy, vx, -1.0, 0.0, 0.0};
        joy_msg.buttons = {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0};
        joy_pub_->publish(joy_msg);

        // Publish Unitree Request
        unitree_api::msg::Request req;
        SportClient sport_req;
        sport_req.Move(req, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);
        go2_request_pub_->publish(req);

        // Check if movement finished
        if (elapsed >= std::max({x_duration_/disp_x_, y_duration_/disp_y_, theta_duration_/disp_theta_}))
        {
            RCLCPP_INFO(this->get_logger(), "Goal Reached!");

            stopRobot();

            auto result = std::make_shared<MoveToGoal::Result>();
            result->success = true;
            active_goal_handle_->succeed(result);
            active_goal_handle_.reset();
        }
    }

    void stopRobot()
    {
        if (timer_) {
            timer_->cancel();
        }

        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.stamp = this->now();
        cmd_vel.header.frame_id = "vehicle";
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.linear.y = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);

        sensor_msgs::msg::Joy joy_msg;
        joy_msg.header.stamp = this->now();
        joy_msg.header.frame_id = "teleop_panel";
        joy_msg.axes = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        joy_msg.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        joy_pub_->publish(joy_msg);

        unitree_api::msg::Request req;
        SportClient sport_req;
        sport_req.StopMove(req);
        go2_request_pub_->publish(req);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRobotServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}