#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono_literals;

class CmdVelToJoy : public rclcpp::Node
{
public:
    CmdVelToJoy()
        : Node("cmd_vel_to_joy")
    {
        // Publisher for /joy topic
        joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);

        // Subscribe to /cmd_vel_robot topic (coming from simulation)
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel_robot", 10,
            std::bind(&CmdVelToJoy::cmdVelCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "CmdVel → Joy Node Started. Listening on /cmd_vel_robot...");
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;

    void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        sensor_msgs::msg::Joy joy_msg;
        joy_msg.header.stamp = this->now();
        joy_msg.header.frame_id = "teleop_panel";

        // Create 8 axes and 11 buttons (typical Joy message structure)

        // Example joystick data
        joy_msg.axes = {msg->twist.angular.z, 0.0, 0.0, 0.0, msg->twist.linear.x, -1.0, 0.0, 0.0}; // index 0: rotat.z, index 3: linear.y, index 4: linear.x, index 5: manual_mode;
        joy_msg.buttons = {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}; 

        // Map linear and angular values from cmd_vel to Joy axes
        // Example mapping:
        //   - index 4: linear.x  (forward/backward)
        //   - index 0: angular.z (rotation)
        // joy_msg.axes[4] = msg->twist.linear.x;   // forward/backward
        // joy_msg.axes[5] = 1;  // turning

        // Optional: enable "manual mode" with a virtual button (index 5)
        
        //if (joy->axes[5] > -0.1) {
        //    checkObstacle = true;
        //} else {
        //    checkObstacle = false;
        //}
        
        // joy_msg.axes[5] = 1.0;  // e.g., 1 = manual control active

        RCLCPP_INFO(this->get_logger(), "Publishing Joy: linear=%.2f angular=%.2f",
                    msg->twist.linear.x, msg->twist.angular.z);

        joy_pub_->publish(joy_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToJoy>());
    rclcpp::shutdown();
    return 0;
}
