#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Joy
import numpy as np
import sys


class TFSubscriber(Node):
    def __init__(self, goal_theta):
        super().__init__('tf_subscriber')
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.listener_callback,
            10
        )

        self.joy_pub = self.create_publisher(Joy, 'joy', 10)

        self.goal_theta = goal_theta
        self.rotation_dir = None   # direction will be set once
        self.get_logger().info(f"Goal orientation: {goal_theta:.3f}")

    def listener_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            current_theta = transform.transform.rotation.z  # (⚠️ see note below)

            # Decide rotation direction only once
            if self.rotation_dir is None:
                self.rotation_dir = self.rotation_direction(current_theta, self.goal_theta)
                self.get_logger().info(f"Initial rotation_dir set to {self.rotation_dir}")

            # Check if reached goal
            if np.isclose(current_theta, self.goal_theta, atol=0.05):
                self.send_joy(0, 0.0)  # stop
                self.get_logger().info("✅ Robot reached goal orientation")
                self.destroy_node()
                rclpy.shutdown()
                return
            else:
                # keep rotating in saved direction
                self.send_joy(self.rotation_dir, 0.5)

    def rotation_direction(self, current_theta, goal_theta):
        diff = (goal_theta - current_theta + 0.5) % 1.0 - 0.5
        return 1 if diff > 0 else -1

    def send_joy(self, orientation=1, speed=0.5):
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.header.frame_id = "teleop_panel"
        joy_msg.axes = [0.0] * 8
        joy_msg.axes[3] = orientation * speed
        self.joy_pub.publish(joy_msg)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run <package_name> tf_subscriber <goal_theta>")
        return

    goal_theta = float(sys.argv[1])
    node = TFSubscriber(goal_theta=goal_theta)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
