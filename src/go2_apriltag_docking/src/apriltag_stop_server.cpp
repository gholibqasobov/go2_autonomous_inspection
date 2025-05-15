#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"
// Action
#include "rclcpp_action/rclcpp_action.hpp"
#include "go2_interfaces/action/move_to_goal.hpp"
#include "go2_interfaces/action/april_tag_stop.hpp"

using namespace std::chrono_literals;
using AprilTagStop = go2_interfaces::action::AprilTagStop;
using MoveToGoal = go2_interfaces::action::MoveToGoal;
using GoalHandleAprilTagStop = rclcpp_action::ServerGoalHandle<AprilTagStop>;

class AprilTagStopServer : public rclcpp::Node {
public:
    AprilTagStopServer() : Node("apriltag_stop_server") {
        apriltag_tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10, 
            std::bind(&AprilTagStopServer::tfCallback, this, std::placeholders::_1));
        move_to_goal_client_ = rclcpp_action::create_client<MoveToGoal>(this, "move_to_goal");
        action_server_ = rclcpp_action::create_server<AprilTagStop>(
            this,
            "apriltag_stop",
            std::bind(&AprilTagStopServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&AprilTagStopServer::handleCancel, this, std::placeholders::_1),
            std::bind(&AprilTagStopServer::handleAccepted, this, std::placeholders::_1)
        );
        last_detection_time_ = this->now();
    }

private:
    rclcpp_action::Server<AprilTagStop>::SharedPtr action_server_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr apriltag_tf_sub_;
    rclcpp_action::Client<MoveToGoal>::SharedPtr move_to_goal_client_;
    rclcpp::Time last_detection_time_;
    std::string apriltag_frame_;
    float forward_meters_;
    bool tag_detected_ = false;
    double tag_timeout_ = 1.0;
    bool action_completed_ = false;

    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const AprilTagStop::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        apriltag_frame_ = goal->apriltag_frame;
        forward_meters_ = goal->forward_meters; 
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleAprilTagStop> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received cancel request for apriltag_stop action");
        // Stop any movement if needed
        moveToGoal(0.0, 0.0, 0.0);
        tag_detected_ = false;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleAprilTagStop> goal_handle) {
        // Create a separate thread to execute the action
        std::thread{std::bind(&AprilTagStopServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleAprilTagStop> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing apriltag_stop action");
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<AprilTagStop::Feedback>();
        auto result = std::make_shared<AprilTagStop::Result>();
        
        // Reset state
        tag_detected_ = false;
        action_completed_ = false;
        
        // Main execution loop
        rclcpp::Rate loop_rate(10);  // 10 Hz
        
        while (rclcpp::ok() && !action_completed_) {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            
            if (tag_detected_) {
                feedback->current_state = "AprilTag detected, robot stopped";
                goal_handle->publish_feedback(feedback);
            } else {
                feedback->current_state = "Waiting for AprilTag detection";
                goal_handle->publish_feedback(feedback);
            }
            
            loop_rate.sleep();
        }
        
        // Action completed - the robot has moved after tag was lost
        feedback->current_state = "Action completed: Robot moved after tag was lost";
        goal_handle->publish_feedback(feedback);
        
        // Set the final result
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded: Robot moved after losing AprilTag");
    }

    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        for (const auto & transform : msg->transforms) {
            if (transform.child_frame_id == apriltag_frame_) {
                last_detection_time_ = this->now();
                tag_detected_ = true;
                // moveToGoal(0.0, 0.0, 0.0);
                // return;
            }
        }
        if (tag_detected_ && (this->now() - last_detection_time_).seconds() > tag_timeout_) {
            tag_detected_ = false;
            RCLCPP_INFO(this->get_logger(), "AprilTag lost, moving robot");
            // std::this_thread::sleep_for(std::chrono::seconds(1));
            moveToGoal(forward_meters_, 0.0, 0.0);
            
            // Set the action as completed after robot movement is initiated
            action_completed_ = true;
        }
    }

    void moveToGoal(double x, double y, double theta) {
        auto goal_msg = MoveToGoal::Goal();
        goal_msg.x = x;
        goal_msg.y = y;
        goal_msg.theta = theta;
        move_to_goal_client_->async_send_goal(goal_msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AprilTagStopServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}