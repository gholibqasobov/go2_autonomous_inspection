#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
using std::placeholders::_1;

class ApriltagPoseSubscriber : public rclcpp::Node
{
  public:
    ApriltagPoseSubscriber()
    : Node("apriltag_pose_subscriber")
    {
      subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 10, std::bind(&ApriltagPoseSubscriber::apriltag_pose_callback, this, _1));
    }

  private:
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;

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
  rclcpp::spin(std::make_shared<ApriltagPoseSubscriber>());
  rclcpp::shutdown();
  return 0;
}