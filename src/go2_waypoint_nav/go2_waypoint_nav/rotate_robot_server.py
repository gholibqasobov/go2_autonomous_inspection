#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Joy
from tf2_msgs.msg import TFMessage
import time
import numpy as np

from go2_interfaces.action import RotateRobotAngle  


class RotateRobotActionServer(Node):

    def __init__(self):
        super().__init__('rotate_robot_action_server')

        self.callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            RotateRobotAngle,
            'rotate_robot',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        
        
        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )

        self._goal_handle = None
        self._goal_canceled = False
        self.current_theta = 0.0
        self.goal_theta = 0.0
        self.rotation_started = False

        self.get_logger().info("RotateRobot Action Server is running...")

    def tf_callback(self, msg: TFMessage):
        """Get current robot orientation from TF messages"""
        for transform in msg.transforms:
            self.current_theta = transform.transform.rotation.z

    def goal_callback(self, goal_request):
        """Called when a goal is received"""
        self.get_logger().info(f"Received goal request: theta={goal_request.theta:.3f}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Called when a goal cancel request comes"""
        self.get_logger().info("Received cancel request")
        self._goal_canceled = True
        return CancelResponse.ACCEPT

    def send_joy(self, orientation=0, speed=0.0):
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.header.frame_id = "teleop_panel"
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 11
        joy_msg.axes[3] = orientation * speed  
        self.joy_pub.publish(joy_msg)
        self.get_logger().info(f"Published Joy command: axes[3]={orientation * speed}")

    def rotation_direction(self, current_theta, goal_theta):
        diff = (goal_theta - current_theta + 0.5) % 1.0 - 0.5
        return 1 if diff > 0 else -1

    async def execute_callback(self, goal_handle):
        """Executes the rotation goal asynchronously"""
        self._goal_handle = goal_handle
        self._goal_canceled = False
        self.rotation_started = False

        result = RotateRobotAngle.Result()

        self.get_logger().info(f'Executing goal: rotate to {goal_handle.request.theta:.3f} rad')

        if goal_handle.is_cancel_requested:
            self.get_logger().info("Goal canceled")
            goal_handle.canceled()
            result.success = False
            return result

        self.goal_theta = goal_handle.request.theta
        
        
        time.sleep(0.1)
        
        
        rotation_dir = self.rotation_direction(self.current_theta, self.goal_theta)
        self.get_logger().info(f"Rotation direction: {rotation_dir}, Current: {self.current_theta:.3f}, Goal: {self.goal_theta:.3f}")

        
        self.send_joy(rotation_dir, 0.5)
        self.rotation_started = True

        feedback = RotateRobotAngle.Feedback()
        
        
        max_iterations = 200  
        iteration = 0
        
        while iteration < max_iterations:
            if goal_handle.is_cancel_requested or self._goal_canceled:
                self.send_joy(0, 0.0)
                goal_handle.canceled()
                self.get_logger().info("Goal canceled - stopping robot")
                result.success = False
                return result

            # Check if we've reached the goal (using same tolerance as your working code)
            if np.isclose(self.current_theta, self.goal_theta, atol=0.03):
                # Stop the robot
                self.send_joy(0, 0.0)
                self.get_logger().info("Robot reached goal orientation")
                result.success = True
                goal_handle.succeed()
                return result

            # Provide feedback
            feedback.current_theta = str(round(self.current_theta, 3))
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"Feedback: current_theta={feedback.current_theta}, goal={self.goal_theta:.3f}")

            time.sleep(0.1)
            iteration += 1

        # If we get here, we hit the safety limit
        self.send_joy(0, 0.0)  # Stop the robot
        self.get_logger().warn("Rotation timeout - stopping robot")
        result.success = False
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    server = RotateRobotActionServer()

    executor = MultiThreadedExecutor()
    executor.add_node(server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()