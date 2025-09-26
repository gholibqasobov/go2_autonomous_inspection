import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Joy

import numpy as np

from go2_interfaces.action import RotateRobotAngle


class RotateRobotActionServer(Node):
    def __init__(self):
        super().__init__('rotate_robot_action_server')

        # Action server
        self._action_server = ActionServer(
            self,
            RotateRobotAngle,
            'rotate_robot',
            self.execute_callback
        )

        # Joy publisher
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)

        # TF subscription
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )

        # State
        self._goal_handle = None
        self._goal_theta = None
        self._rotation_dir = None
        self._current_theta = None

    def tf_callback(self, msg: TFMessage):
        if self._goal_handle is None:
            return  # no active goal

        for transform in msg.transforms:
            self._current_theta = transform.transform.rotation.z  
            break

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f"Received goal: Rotate to theta={goal_handle.request.theta:.3f}")
        self._goal_handle = goal_handle
        self._goal_theta = goal_handle.request.theta
        self._rotation_dir = None

        feedback_msg = RotateRobotAngle.Feedback()

        # Loop until goal reached or canceled
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.send_joy(0, 0.0)
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                return RotateRobotAngle.Result(success=False)

            if self._current_theta is None:
                await rclpy.sleep(0.1)  # wait for TF
                continue

            # Decide rotation direction once
            if self._rotation_dir is None:
                self._rotation_dir = self.rotation_direction(self._current_theta, self._goal_theta)
                self.get_logger().info(f"Initial rotation_dir set to {self._rotation_dir}")

            # Send feedback
            feedback_msg.current_theta = str(self._current_theta)
            goal_handle.publish_feedback(feedback_msg)

            # Check if reached goal
            if np.isclose(self._current_theta, self._goal_theta, atol=0.05):
                self.send_joy(0, 0.0)
                self.get_logger().info("Robot reached goal orientation")
                goal_handle.succeed()
                return RotateRobotAngle.Result(success=True)

            # Keep rotating
            self.send_joy(self._rotation_dir, 0.5)

            await rclpy.sleep(0.1)

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
    action_server = RotateRobotActionServer()

    executor = MultiThreadedExecutor()
    rclpy.spin(action_server, executor=executor)


if __name__ == '__main__':
    main()
