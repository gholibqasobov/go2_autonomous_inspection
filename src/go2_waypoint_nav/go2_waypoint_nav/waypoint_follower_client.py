import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from go2_interfaces.action import WaypointFollower

class WaypointFollowerClient(Node):

    def __init__(self):
        super().__init__('waypoint_follower_client')
        self._action_client = ActionClient(
            self, 
            WaypointFollower, 
            'waypoint_follower'
        )

    def send_goal(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        # Create a PoseStamped message for the waypoint
        waypoint = PoseStamped()
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.header.frame_id = 'map'  # Adjust frame_id as needed
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.position.z = z
        waypoint.pose.orientation.x = qx
        waypoint.pose.orientation.y = qy
        waypoint.pose.orientation.z = qz
        waypoint.pose.orientation.w = qw

        # Create goal message
        goal_msg = WaypointFollower.Goal()
        goal_msg.waypoint = waypoint

        # Wait for action server
        self._action_client.wait_for_server()

        # Send goal
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

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.current_position}')


def main(args=None):
    rclpy.init(args=args)
    
    # Create client node
    action_client = WaypointFollowerClient()
    
    # Send a sample waypoint (x, y, z)
    action_client.send_goal(1.0, 2.0, 0.0)
    
    # Spin until result is received
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()