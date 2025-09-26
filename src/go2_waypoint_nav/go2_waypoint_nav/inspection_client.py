import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from go2_interfaces.action import WaypointFollower  # Your custom action


class SimpleWaypointClient(Node):
    def __init__(self):
        super().__init__('simple_waypoint_client')
        self._action_client = ActionClient(self, WaypointFollower, 'waypoint_follower')

    def send_goal(self, x, y, z):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create a PointStamped goal message
        waypoint = PointStamped()
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.header.frame_id = 'map'  # or your appropriate frame
        waypoint.point.x = x
        waypoint.point.y = y
        waypoint.point.z = z

        goal_msg = WaypointFollower.Goal()
        goal_msg.waypoint = waypoint

        self.get_logger().info(f'Sending goal: x={x}, y={y}, z={z}')
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
        self.get_logger().info(f'Feedback: {feedback.current_position}')

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().info('Failed to reach the goal.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    client = SimpleWaypointClient()

    # Send a sample goal point
    client.send_goal(3.0, 3.0, 0.3659999966621399)

    rclpy.spin(client)


if __name__ == '__main__':
    main()



"""

actions

- navigate
- rotate
- inspect

"""