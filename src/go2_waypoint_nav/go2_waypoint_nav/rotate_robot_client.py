#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import sys
from go2_interfaces.action import RotateRobotAngle


class RotateRobotActionClient(Node):

    def __init__(self):
        super().__init__('rotate_robot_action_client')
        self._action_client = ActionClient(self, RotateRobotAngle, 'rotate_robot')

    def send_goal(self, theta):
        self.get_logger().info(f'Sending goal to rotate robot to {theta:.3f} radians')

        goal_msg = RotateRobotAngle.Goal()
        goal_msg.theta = theta

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: current_theta = {feedback.current_theta}')

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Rotation succeeded!')
        else:
            self.get_logger().info('Rotation failed or was canceled.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run <package_name> tf_subscriber <goal_theta>")
        return
    
    goal_theta = float(sys.argv[1])
    

    action_client = RotateRobotActionClient()

    # Example: rotate robot by 0.5 radians
    action_client.send_goal(goal_theta)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
