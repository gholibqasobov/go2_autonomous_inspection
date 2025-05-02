import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from go2_interfaces.action import AprilTagDocking, PushElevatorButton, MoveToGoal


class ElevatorControllerClient(Node):
    def __init__(self):
        super().__init__('elevator_controller_client')
        callback_group = ReentrantCallbackGroup()

        # Action clients
        self._clients = {
            'docking': ActionClient(self, AprilTagDocking, 'apriltag_docking', callback_group=callback_group),
            'button': ActionClient(self, PushElevatorButton, 'push_elevator_button', callback_group=callback_group),
            'move': ActionClient(self, MoveToGoal, 'move_to_goal', callback_group=callback_group)
        }

        self.get_logger().info('Elevator Controller Client initialized')
        # sequence storage
        self._sequence = []
        self._current_step = 0

    def run_sequence(self, steps):
        """
        Run a list of sequential actions. Each step is a dict with 'type' and its params.
        Supported types: 'docking', 'button', 'move'.
        """
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

        # dispatch
        if action_type == 'docking':
            self.send_docking_goal(step['apriltag_frame'], self._step_done_cb)
        elif action_type == 'button':
            self.send_button_push_goal(step['floor_number'], step['apriltag_frame'], self._step_done_cb)
        elif action_type == 'move':
            self.send_move_robot_goal(step.get('x', 0), step.get('y', 0), step.get('theta', 0), self._step_done_cb)
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

    # -- action wrappers with callback on completion --
    def send_docking_goal(self, apriltag_frame, done_cb):
        self.get_logger().info('Waiting for Docking server...')
        client = self._clients['docking']
        client.wait_for_server()
        goal = AprilTagDocking.Goal(apriltag_frame=apriltag_frame)
        fut = client.send_goal_async(goal, feedback_callback=self._docking_feedback)
        fut.add_done_callback(lambda f: self._on_docking_response(f, done_cb))

    def _on_docking_response(self, future, done_cb):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().info('Docking rejected')
            done_cb(False)
            return
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(lambda f: done_cb(f.result().status == 4))

    def _docking_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'Docking state: {fb.current_state}, distance: {fb.distance_to_goal}')

    def send_button_push_goal(self, floor_number, apriltag_frame, done_cb):
        self.get_logger().info('Waiting for Button Push server...')
        client = self._clients['button']
        client.wait_for_server()
        goal = PushElevatorButton.Goal(floor_number=floor_number, apriltag_frame=apriltag_frame)
        fut = client.send_goal_async(goal, feedback_callback=self._button_feedback)
        fut.add_done_callback(lambda f: self._on_button_response(f, done_cb))

    def _on_button_response(self, future, done_cb):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().info('Button push rejected')
            done_cb(False)
            return
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(lambda f: done_cb(f.result().status == 4))

    def _button_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'Button push state: {fb.current_state}')

    def send_move_robot_goal(self, x, y, theta, done_cb):
        self.get_logger().info('Waiting for Move Robot server...')
        client = self._clients['move']
        client.wait_for_server()
        goal = MoveToGoal.Goal(x=int(x), y=int(y), theta=int(theta))
        fut = client.send_goal_async(goal, feedback_callback=self._move_feedback)
        fut.add_done_callback(lambda f: self._on_move_response(f, done_cb))

    def _on_move_response(self, future, done_cb):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().info('Move rejected')
            done_cb(False)
            return
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(lambda f: done_cb(f.result().status == 4))

    def _move_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'Move state: {fb.current_state}')

    def cancel_all_goals(self):
        self.get_logger().info('Canceling all active goals...')
        for client in self._clients.values():
            client.cancel_all_goals()


def main(args=None):
    rclpy.init(args=args)
    node = ElevatorControllerClient()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Define your sequence
    sequence = [
        {'type': 'docking', 'apriltag_frame': 'elevator_panel'},
        {'type': 'button', 'floor_number': 2, 'apriltag_frame': 'elevator_panel'},
        {'type': 'move', 'x': 0, 'y': 1, 'theta': 0}
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
