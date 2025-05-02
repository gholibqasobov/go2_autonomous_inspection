import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from go2_interfaces.action import AprilTagDocking, PushElevatorButton, MoveToGoal
import sys


class ElevatorControllerClient(Node):
    def __init__(self):
        super().__init__('elevator_controller_client')
        
        # Use a reentrant callback group to allow concurrent processing
        callback_group = ReentrantCallbackGroup()

        # Docking Action Client
        self._docking_action_client = ActionClient(
            self,
            AprilTagDocking,
            'apriltag_docking',
            callback_group=callback_group
        )

        # Roarm Button Press Action Client
        self._roarm_action_client = ActionClient(
            self,
            PushElevatorButton,
            'push_elevator_button',
            callback_group=callback_group
        )

        # Move Robot Action Client
        self._move_robot_action_client = ActionClient(
            self,
            MoveToGoal,
            'move_to_goal',
            callback_group=callback_group
        )

        # Action Sequence Storage
        self._sequence = []
        self._current_step = 0


        self.get_logger().info('Elevator Controller Client has been initialized')
    
    def run_sequence(self, steps):
        self._sequence = steps
        self._current_step = 0
        self._run_next_step()
    

    def _run_next_step(self):
        if self._current_step >= len(self._sequence):
            self.get_logger().info('Sequence Completed')
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

    # AprilTag Docking Client Methods
    def send_docking_goal(self, apriltag_frame):
        """Send a goal to dock with the specified AprilTag"""
        self.get_logger().info('Waiting for Docking action server...')
        
        # Wait for the action server to become available
        self._docking_action_client.wait_for_server()
        
        # Create goal message
        goal_msg = AprilTagDocking.Goal()
        goal_msg.apriltag_frame = apriltag_frame
        
        self.get_logger().info(f'Sending goal to dock with AprilTag: {apriltag_frame}')
        
        # Send the goal
        self._send_docking_goal_future = self._docking_action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.docking_feedback_callback
        )
        
        # Add callback for goal response
        self._send_docking_goal_future.add_done_callback(self.docking_goal_response_callbac)


    def docking_goal_response_callback(self, future, done_cb):
        """Callback when the docking goal is accepted or rejected"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Docking goal rejected')
            return
        
        self.get_logger().info('Docking goal accepted')
        
        # Request the result
        self._get_docking_result_future = goal_handle.get_result_async()
        self._get_docking_result_future.add_done_callback(self.get_docking_result_callback)

  
    
    def get_docking_result_callback(self, future):
        """Callback when the docking action is completed"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Docking succeeded! Success: {result.success}')
        elif status == 2:  # CANCELED
            self.get_logger().info('Docking was canceled')
        elif status == 3:  # ABORTED
            self.get_logger().info('Docking was aborted')
        else:
            self.get_logger().info(f'Docking has unknown state: {status}')
            
        # Shutdown the node
        rclpy.shutdown()

    
    def docking_feedback_callback(self, feedback_msg):
        """Callback for Docking feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current Docking state: {feedback.current_state}')
        self.get_logger().info(f'Docking Distance info: {feedback.distance_to_goal}')



    # Elevator button push Client Methods
    def send_button_push_goal(self, floor_number, apriltag_frame):
        """Send a goal to push an elevator button for the specified floor"""
        self.get_logger().info(f'Waiting for elevator button pusher action server...')
        
        # Wait for the action server to become available
        self._roarm_action_client.wait_for_server()
        
        # Create goal message
        goal_msg = PushElevatorButton.Goal()
        goal_msg.floor_number = floor_number
        goal_msg.apriltag_frame = apriltag_frame
        
        self.get_logger().info(f'Sending goal to push button for floor {floor_number}')
        
        # Send the goal
        self._send_button_push_goal_future = self._roarm_action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.button_push_feedback_callback
        )
        
        # Add callback for goal response
        self._send_button_push_goal_future.add_done_callback(self.button_push_goal_response_callback)


    def button_push_goal_response_callback(self, future):
        """Callback when the button push goal is accepted or rejected"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Elevator Button Push Goal rejected')
            return
        
        self.get_logger().info('Elevator Button Push Goal accepted')
        
        # Request the result
        self._get_button_push_result_future = goal_handle.get_result_async()
        self._get_button_push_result_future.add_done_callback(self.get_button_push_result_callback)

    def get_button_push_result_callback(self, future):
        """Callback when the Push Button action is completed"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Push Button Goal succeeded! Success: {result.success}')
        elif status == 2:  # CANCELED
            self.get_logger().info('Push Button Goal was canceled')
        elif status == 3:  # ABORTED
            self.get_logger().info('Push Button Goal was aborted')
        else:
            self.get_logger().info(f'Push Button Goal has unknown state: {status}')
            
        # Shutdown the node
        rclpy.shutdown()
    
    def button_push_feedback_callback(self, feedback_msg):
        """Callback for Push Button feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current Elevator Push Button state: {feedback.current_state}')



    # Move Robot Client Methods
    def send_move_robot_goal(self, x, y, theta):
        
        self.get_logger().info('Waiting for Move Robot action server...')
        
        # Wait for the action server to become available
        self._move_robot_action_client.wait_for_server()
        
        # Create goal message
        goal_msg = MoveToGoal.Goal()
        goal_msg.x = int(x)
        goal_msg.y = int(y)
        goal_msg.theta = int(theta)

        self.get_logger().info(f"Sending goal to MoveRobot x: {x}, y: {y}, theta: {theta}")
    
        # Send the goal
        self._send_move_robot_goal_future = self._move_robot_action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.move_robot_feedback_callback
        )
        
        # Add callback for goal response
        self._send_move_robot_goal_future.add_done_callback(self.move_robot_goal_response_callback)

    def move_robot_goal_response_callback(self, future):
        """Callback when the move robot goal is accepted or rejected"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Move Robot Goal rejected')
            return
        
        self.get_logger().info('Move Robot Goal accepted')
        
        # Request the result
        self._get_move_robot_result_future = goal_handle.get_result_async()
        self._get_move_robot_result_future.add_done_callback(self.get_move_robot_result_callback)

    def get_move_robot_result_callback(self, future):
        """Callback when the Move Robot action is completed"""
        result = future.result().result
        status = future.result().status

        self.get_logger().info(f"Move Robot Status: {status}")
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Move Robot Goal succeeded! Success: {result.success}')
        elif status == 2:  # CANCELED
            self.get_logger().info('Move Robot Goal was canceled')
        elif status == 3:  # ABORTED
            self.get_logger().info('Move Robot Goal was aborted')
        else:
            self.get_logger().info(f'Move Robot Goal has unknown state: {status}')
            
        # Shutdown the node
        self.get_logger().info('Move Robot Action completed')
        rclpy.shutdown()
    
    def move_robot_feedback_callback(self, feedback_msg):
        """Callback for move robot feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Move Robot Current state: {feedback.current_state}')



    # General Function to Cancel a Goal

    def cancel_all_goals(self):
        """Cancel any active goals"""
        self.get_logger().info('Canceling all active goals')
        
        # Cancel docking if active
        if hasattr(self, '_send_docking_goal_future') and self._send_docking_goal_future.done():
            goal_handle = self._send_docking_goal_future.result()
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
                self.get_logger().info('Docking cancel request sent')
        
        # Cancel button pushing if active
        if hasattr(self, '_send_button_push_goal_future') and self._send_button_push_goal_future.done():
            goal_handle = self._send_button_push_goal_future.result()
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
                self.get_logger().info('Button pushing cancel request sent')

        # Cancel robot moving if active
        if hasattr(self, '_send_move_robot_goal_future') and self._send_move_robot_goal_future.done():
            goal_handle = self._send_move_robot_goal_future.result()
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
                self.get_logger().info('Move Robot cancel request sent')

        


def main(args=None):
    rclpy.init(args=args)
    
    client_node = ElevatorControllerClient()
    
    # Use MultiThreadedExecutor for handling callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(client_node)
    
  
    executor.spin()
    client_node.destroy_node()


if __name__ == '__main__':
    main()