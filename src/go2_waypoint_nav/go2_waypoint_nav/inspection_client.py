import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PointStamped
from go2_interfaces.action import WaypointFollower, RotateRobotAngle, DetectClosest
import yaml

class CombinedActionClient(Node):
    def __init__(self):
        super().__init__('combined_action_client')
        callback_group = ReentrantCallbackGroup()

        # Action clients dictionary including DetectClosest
        self._action_clients = {
            'waypoint_follower': ActionClient(self, WaypointFollower, 'waypoint_follower', callback_group=callback_group),
            'rotate_robot': ActionClient(self, RotateRobotAngle, 'rotate_robot', callback_group=callback_group),
            'detect_closest': ActionClient(self, DetectClosest, '/damaged_object_tracker_server/detect_closest', callback_group=callback_group)
        }

        self.get_logger().info('Inspection Client initialized')

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

        elif action_type == 'detect_closest':
            class_name = step.get('class_name', '')
            timeout = step.get('timeout', 20.0)
            confirm_frames = step.get('confirm_frames', 3)
            require_depth = step.get('require_depth', False)
            self.send_detect_closest_goal(class_name, timeout, confirm_frames, require_depth, self._step_done_cb)

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

    # -- DetectClosest Action --
    def send_detect_closest_goal(self, class_name, timeout_s, confirm_frames, require_depth, done_cb):
        self.get_logger().info(f'Waiting for DetectClosest server to detect class "{class_name}"...')
        client = self._action_clients['detect_closest']
        client.wait_for_server()

        goal_msg = DetectClosest.Goal()
        goal_msg.class_name = str(class_name)
        goal_msg.timeout_s = float(timeout_s)
        goal_msg.confirm_frames = int(confirm_frames)
        goal_msg.require_depth = bool(require_depth)

        fut = client.send_goal_async(goal_msg, feedback_callback=self._detect_closest_feedback)
        fut.add_done_callback(lambda f: self._on_detect_closest_response(f, done_cb))

    def _on_detect_closest_response(self, future, done_cb):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('DetectClosest goal rejected')
            done_cb(False)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._detect_closest_result_callback(f, done_cb))

    def _detect_closest_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        closest = f"{fb.closest_px_dist:.1f}px" if fb.closest_px_dist >= 0 else "N/A"
        self.get_logger().info(
            f'DetectClosest feedback: class={fb.active_class} closest={closest} '
            f'candidates={fb.candidates} camera_moving={fb.camera_moving} depth_ready={fb.depth_ready}'
        )

    def _detect_closest_result_callback(self, future, done_cb):
        result = future.result().result
        success = getattr(result, 'success', False)
        reason = getattr(result, 'reason', '')
        self.get_logger().info(f'DetectClosest result success={success} reason="{reason}"')
        done_cb(success)


def main(args=None):
    rclpy.init(args=args)
    node = CombinedActionClient()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    with open("/home/qasob/go2_autonomous_inspection/src/go2_waypoint_nav/navigation_path/test_waypoint.yaml") as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return

    waypoints = []
    orientations = []

    for i in range(len(data['waypoints'])):
        orientation = data['waypoints'][i]['pose']['orientation']
        position = data['waypoints'][i]['pose']['position']

        theta = orientation['z']  # or orientation['w'] if that is the correct yaw
        x, y, z = position['x'], position['y'], position['z']

        waypoints.append({'type': 'waypoint_follower', 'x': x, 'y': y, 'z': z})
        orientations.append({'type': 'rotate_robot', 'theta': theta})

    combined_sequence = []
    for wp, ori in zip(waypoints, orientations):
        combined_sequence.append(wp)
        combined_sequence.append(ori)

    # Example adding DetectClosest step somewhere in sequence:
    combined_sequence.append({'type': 'detect_closest', 'class_name': 'crack', 'timeout': 20.0, 'confirm_frames': 3, 'require_depth': False})

    node.run_sequence(combined_sequence)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
