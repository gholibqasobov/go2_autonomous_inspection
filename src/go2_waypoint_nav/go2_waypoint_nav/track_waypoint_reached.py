#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import numpy as np


class TFSubscriber(Node):

    def __init__(self):
        super().__init__('tf_subscriber')

        # Subscribe to the /tf topic
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.listener_callback,
            10
        )

        # Store current and last norm
        self.normalized = None
        self.last_value = None

        # Timer to check every 1.5 seconds
        self.timer = self.create_timer(1.5, self.check_reached_goal)

    def listener_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            t = transform.transform.translation
            r = transform.transform.rotation

            # Use translation + w component of quaternion (theta)
            self.normalized = self.norm(t.x, t.y, t.z, r.z, r.w)

            self.get_logger().info(
                f"Translation: x={t.x:.3f}, y={t.y:.3f}, z={t.z:.3f}\n"
                f"Rotation (quaternion): x={r.x:.3f}, y={r.y:.3f}, z={r.z:.3f}, w={r.w:.3f}\n"
                f"Normalized: {self.normalized:.6f}"
            )

    def norm(self, x, y, z, rz, rw):
        return np.linalg.norm([x, y, z, rz, rw])

    def check_reached_goal(self):
        """Check every 1.5 seconds if the robot has stopped moving."""
        if self.normalized is None:
            return  # no data received yet

        if self.last_value is None:
            self.last_value = self.normalized
            return

        if np.isclose(self.last_value, self.normalized, atol=1e-6):
            self.get_logger().info("Robot reached goal position.")
            # Optionally stop the timer once goal is reached
            self.timer.cancel()
            rclpy.shutdown()
        else:
            self.last_value = self.normalized


def main(args=None):
    rclpy.init(args=args)
    node = TFSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
