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
        # Subscribe to the /tf topic
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.listener_callback,
            10  # QoS history depth
        )

        self.joy_pub = self.create_publisher(Joy, 'joy', 10)

        self.subscription  # prevent unused variable warning
        self.goal_theta = goal_theta


    def listener_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            current_theta = transform.transform.rotation.z
            rotation_dir = self.rotation_direction(current_theta, self.goal_theta)

        if np.isclose(current_theta, self.goal_theta, atol=0.05):
            self.send_joy(1, 0.0)
            self.get_logger().info("Robot reached goal orientation")
            rclpy.shutdown()
        else:

            self.send_joy(rotation_dir, 0.5)

            


    def rotation_direction(self, current_theta, goal_theta):
        diff = (goal_theta - current_theta + 0.5) % 1.0 - 0.5

        if diff > 0:
            return -1
        else:
            return 1
    
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
        return
    
    goal_theta = float(sys.argv[1])


    node = TFSubscriber(goal_theta=goal_theta)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
