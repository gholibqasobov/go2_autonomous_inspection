import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PointStamped
from go2_interfaces.action import WaypointFollower, RotateRobotAngle


class CombinedActionClient(Node):
    def __init__(self):
        super().__init__('combined_action_client')
        callback_group = ReentrantCallbackGroup()

        # Action clients dictionary
        self._action_clients = {
            'waypoint_follower': ActionClient(self, WaypointFollower, 'waypoint_follower', callback_group=callback_group),
            'rotate_robot': ActionClient(self, RotateRobotAngle, 'rotate_robot', callback_group=callback_group)
        }

        self.get_logger().info('Combined Action Client initialized')

        # Sequence of steps (list of dicts)
        self._sequence = []
        self._current_step = 0

    def run_sequence(self, steps):
        self._sequence = steps
        self._current_step = 0
        self._run_next_step()

    def _run_next_step(self):
        if self._current_step >= len(self._sequence):
            self.get_logger().info('Sequence completed')
            rclpy.shutdown()
            return

        step = self._sequence[self._current_step]
        action_type = step['type']

        if action_type == 'waypoint_follower':
            x = step.get('x', 0.0)
            y = step.get('y', 0.0)
            z = step.get('z', 0.0)
            self.send_waypoint_goal(x, y, z, self._step_done_cb)

        elif action_type == 'rotate_robot':
            theta = step.get('theta', 0.0)
            self.send_rotate_goal(theta, self._step_done_cb)

        else:
            self.get_logger().error(f'Unknown step type: {action_type}')
            rclpy.shutdown()

    def _step_done_cb(self, success: bool):
        if not success:
            self.get_logger().error(f'Step {self._current_step} failed, aborting sequence')
            rclpy.shutdown()
            return
        self.get_logger().info(f'Step {self._current_step} succeeded')
        self._current_step += 1
        self._run_next_step()

    # -- WaypointFollower Action --
    def send_waypoint_goal(self, x, y, z, done_cb):
        self.get_logger().info('Waiting for WaypointFollower server...')
        client = self._action_clients['waypoint_follower']
        client.wait_for_server()

        # Construct PointStamped
        waypoint = PointStamped()
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.header.frame_id = 'map'
        waypoint.point.x = x
        waypoint.point.y = y
        waypoint.point.z = z

        goal_msg = WaypointFollower.Goal()
        goal_msg.waypoint = waypoint

        fut = client.send_goal_async(goal_msg, feedback_callback=self._waypoint_feedback)
        fut.add_done_callback(lambda f: self._on_waypoint_response(f, done_cb))

    def _on_waypoint_response(self, future, done_cb):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Waypoint goal rejected')
            done_cb(False)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._waypoint_result_callback(f, done_cb))

    def _waypoint_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'WaypointFollower feedback: current_position = {fb.current_position}')

    def _waypoint_result_callback(self, future, done_cb):
        result = future.result().result
        success = getattr(result, 'success', False)
        self.get_logger().info(f'WaypointFollower result success: {success}')
        done_cb(success)

    # -- RotateRobot Action --
    def send_rotate_goal(self, theta, done_cb):
        self.get_logger().info(f'Waiting for RotateRobot server to rotate to {theta:.3f} radians...')
        client = self._action_clients['rotate_robot']
        client.wait_for_server()

        goal_msg = RotateRobotAngle.Goal()
        goal_msg.theta = theta

        fut = client.send_goal_async(goal_msg, feedback_callback=self._rotate_feedback)
        fut.add_done_callback(lambda f: self._on_rotate_response(f, done_cb))

    def _on_rotate_response(self, future, done_cb):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('RotateRobot goal rejected')
            done_cb(False)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._rotate_result_callback(f, done_cb))

    def _rotate_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'RotateRobot feedback: current_theta = {fb.current_theta}')

    def _rotate_result_callback(self, future, done_cb):
        result = future.result().result
        success = getattr(result, 'success', False)
        if success:
            self.get_logger().info('Rotation succeeded!')
        else:
            self.get_logger().info('Rotation failed or was canceled.')
        done_cb(success)


def main(args=None):
    rclpy.init(args=args)
    node = CombinedActionClient()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Example sequence combining both actions
    sequence = [
        {'type': 'waypoint_follower', 'x': 1.0, 'y': 2.0, 'z': 0.0},
        {'type': 'rotate_robot', 'theta': 0.5},
        {'type': 'waypoint_follower', 'x': 3.0, 'y': 1.0, 'z': 0.0},
        {'type': 'rotate_robot', 'theta': -0.5}
    ]

    node.run_sequence(sequence)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
