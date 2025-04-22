import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from go2_interfaces.action import AprilTagDocking, PushElevatorButton


class ElevatorControllerNode(Node):
    def __init__(self):
        super().__init__('elevator_controller_node')
        
        # Use a reentrant callback group to allow concurrent processing
        self.callback_group = ReentrantCallbackGroup()
        
        # Create action clients
        self._docking_action_client = ActionClient(
            self,
            AprilTagDocking,
            'apriltag_docking',
            callback_group=self.callback_group
        )
        
        self._elevator_action_client = ActionClient(
            self,
            PushElevatorButton,
            'push_elevator_button',
            callback_group=self.callback_group
        )
        
        # State tracking
        self._docking_complete = False
        self._button_push_complete = False
        self._current_operation = "INITIALIZING"
        
        self.get_logger().info('Elevator Controller Node initialized')
    
    def start_sequence(self, docking_tag="elevator_docking", floor_number=0):
        """Start the elevator control sequence"""
        self.docking_tag = docking_tag
        self.floor_number = floor_number
        
        self.get_logger().info(f'Starting elevator control sequence')
        self.get_logger().info(f'Target: AprilTag "{docking_tag}", Floor: {floor_number}')
        
        # Begin with docking
        self._current_operation = "DOCKING"
        self.start_docking(docking_tag)
    
    def start_docking(self, apriltag_frame):
        """Start the docking process"""
        self.get_logger().info('Waiting for docking action server...')
        
        # Wait for the action server to become available
        self._docking_action_client.wait_for_server()
        
        # Create goal message
        goal_msg = AprilTagDocking.Goal()
        goal_msg.apriltag_frame = apriltag_frame
        
        self.get_logger().info(f'Sending goal to dock with AprilTag: {apriltag_frame}')
        
        # Send the goal
        self._docking_goal_future = self._docking_action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.docking_feedback_callback
        )
        
        # Add callback for goal response
        self._docking_goal_future.add_done_callback(self.docking_goal_response_callback)
    
    def docking_goal_response_callback(self, future):
        """Callback when the docking goal is accepted or rejected"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Docking goal rejected')
            self._current_operation = "FAILED"
            return
        
        self.get_logger().info('Docking goal accepted')
        
        # Request the result
        self._docking_result_future = goal_handle.get_result_async()
        self._docking_result_future.add_done_callback(self.docking_result_callback)
    
    def docking_result_callback(self, future):
        """Callback when the docking action is completed"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Docking succeeded! Success: {result.success}')
            if result.success:
                self._docking_complete = True
                self._current_operation = "BUTTON_PUSHING"
                # Now that docking is complete, start button pushing
                self.start_button_pushing(self.floor_number, self.docking_tag)
            else:
                self.get_logger().error('Docking reported success=False')
                self._current_operation = "FAILED"
        elif status == 2:  # CANCELED
            self.get_logger().error('Docking was canceled')
            self._current_operation = "FAILED"
        elif status == 3:  # ABORTED
            self.get_logger().error('Docking was aborted')
            self._current_operation = "FAILED"
        else:
            self.get_logger().error(f'Docking has unknown state: {status}')
            self._current_operation = "FAILED"
    
    def docking_feedback_callback(self, feedback_msg):
        """Callback for docking feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Docking state: {feedback.current_state}')
        self.get_logger().info(f'Distance info: {feedback.distance_to_goal}')
    
    def start_button_pushing(self, floor_number, apriltag_frame):
        """Start the button pushing process"""
        self.get_logger().info('Waiting for elevator button action server...')
        
        # Wait for the action server to become available
        self._elevator_action_client.wait_for_server()
        
        # Create goal message
        goal_msg = PushElevatorButton.Goal()
        goal_msg.floor_number = floor_number
        goal_msg.apriltag_frame = apriltag_frame
        
        self.get_logger().info(f'Sending goal to push button for floor {floor_number}')
        
        # Send the goal
        self._elevator_goal_future = self._elevator_action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.button_feedback_callback
        )
        
        # Add callback for goal response
        self._elevator_goal_future.add_done_callback(self.button_goal_response_callback)
    
    def button_goal_response_callback(self, future):
        """Callback when the button pushing goal is accepted or rejected"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Button pushing goal rejected')
            self._current_operation = "FAILED"
            return
        
        self.get_logger().info('Button pushing goal accepted')
        
        # Request the result
        self._elevator_result_future = goal_handle.get_result_async()
        self._elevator_result_future.add_done_callback(self.button_result_callback)
    
    def button_result_callback(self, future):
        """Callback when the button pushing action is completed"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Button pushing succeeded! Success: {result.success}')
            if result.success:
                self._button_push_complete = True
                self._current_operation = "COMPLETED"
                self.get_logger().info('Full elevator control sequence completed successfully')
            else:
                self.get_logger().error('Button pushing reported success=False')
                self._current_operation = "PARTIAL_FAILURE"
        elif status == 2:  # CANCELED
            self.get_logger().error('Button pushing was canceled')
            self._current_operation = "PARTIAL_FAILURE"
        elif status == 3:  # ABORTED
            self.get_logger().error('Button pushing was aborted')
            self._current_operation = "PARTIAL_FAILURE"
        else:
            self.get_logger().error(f'Button pushing has unknown state: {status}')
            self._current_operation = "PARTIAL_FAILURE"
        
        # Sequence is complete (either success or failure)
        self.report_final_status()
    
    def button_feedback_callback(self, feedback_msg):
        """Callback for button pushing feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Button pushing state: {feedback.current_state}')
    
    def report_final_status(self):
        """Report the final status of the entire operation"""
        self.get_logger().info('=============================================')
        self.get_logger().info('Elevator Control Sequence Complete')
        self.get_logger().info(f'Final status: {self._current_operation}')
        self.get_logger().info(f'Docking completed: {self._docking_complete}')
        self.get_logger().info(f'Button pushing completed: {self._button_push_complete}')
        self.get_logger().info('=============================================')
        
        # Shutdown after sequence is done
        rclpy.shutdown()
    
    def cancel_all_goals(self):
        """Cancel any active goals"""
        self.get_logger().info('Canceling all active goals')
        
        # Cancel docking if active
        if hasattr(self, '_docking_goal_future') and self._docking_goal_future.done():
            goal_handle = self._docking_goal_future.result()
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
                self.get_logger().info('Docking cancel request sent')
        
        # Cancel button pushing if active
        if hasattr(self, '_elevator_goal_future') and self._elevator_goal_future.done():
            goal_handle = self._elevator_goal_future.result()
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
                self.get_logger().info('Button pushing cancel request sent')


def main(args=None):
    rclpy.init(args=args)
    
    # Create the controller node
    controller_node = ElevatorControllerNode()
    
    # Use MultiThreadedExecutor for handling callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(controller_node)
    
    # Start the sequence with default parameters
    # AprilTag frame: 'elevator_docking', Floor: 0
    controller_node.start_sequence()
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        controller_node.get_logger().info('Keyboard interrupt received, canceling all goals')
        controller_node.cancel_all_goals()
        executor.spin_once()  # Process any remaining callbacks
    finally:
        controller_node.destroy_node()


if __name__ == '__main__':
    main()