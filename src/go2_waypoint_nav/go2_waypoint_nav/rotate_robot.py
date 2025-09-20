#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


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
        self.subscription  # prevent unused variable warning
        self.goal_theta = goal_theta


    def listener_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            current_theta = transform.transform.rotation.w


def main(args=None):
    rclpy.init(args=args)
    node = TFSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
