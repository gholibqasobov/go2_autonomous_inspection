#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class TFSubscriber(Node):
    def __init__(self):
        super().__init__('tf_subscriber')
        # Subscribe to the /tf topic
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.listener_callback,
            10  # QoS history depth
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            t = transform.transform.translation
            r = transform.transform.rotation

            self.get_logger().info(
                f"Translation: x={t.x:.3f}, y={t.y:.3f}, z={t.z:.3f}\n"
                f"Rotation (quaternion): x={r.x:.3f}, y={r.y:.3f}, z={r.z:.3f}, w={r.w:.3f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TFSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
