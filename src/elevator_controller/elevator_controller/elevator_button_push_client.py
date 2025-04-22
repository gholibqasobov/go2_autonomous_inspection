#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from go2_interfaces.action import PushElevatorButton
import sys


class ElevatorButtonPushClient(Node):
    def __init__(self):
        super().__init__('elevator_button_push_client')
        
        # Use a reentrant callback group to allow concurrent processing
        callback_group = ReentrantCallbackGroup()
        
        self._roarm_action_client = ActionClient(
            self,
            PushElevatorButton,
            'push_elevator_button',
            callback_group=callback_group
        )
        
        self.get_logger().info('Elevator Button Push Client has been initialized')
    
    def send_goal(self, floor_number, apriltag_frame):
        """Send a goal to push an elevator button for the specified floor"""
        self.get_logger().info(f'Waiting for action server...')
        
        # Wait for the action server to become available
        self._roarm_action_client.wait_for_server()
        
        # Create goal message
        goal_msg = PushElevatorButton.Goal()
        goal_msg.floor_number = floor_number
        goal_msg.apriltag_frame = apriltag_frame
        
        self.get_logger().info(f'Sending goal to push button for floor {floor_number}')
        
        # Send the goal
        self._send_goal_future = self._roarm_action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        
        # Add callback for goal response
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Callback when the goal is accepted or rejected"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        
        # Request the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Callback when the action is completed"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Goal succeeded! Success: {result.success}')
        elif status == 2:  # CANCELED
            self.get_logger().info('Goal was canceled')
        elif status == 3:  # ABORTED
            self.get_logger().info('Goal was aborted')
        else:
            self.get_logger().info(f'Goal has unknown state: {status}')
            
        # Shutdown the node
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        """Callback for feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current state: {feedback.current_state}')
    
    def cancel_goal(self):
        """Cancel the current goal"""
        self.get_logger().info('Canceling current goal')
        
        # Check if a goal is active
        if hasattr(self, '_send_goal_future') and self._send_goal_future.done():
            goal_handle = self._send_goal_future.result()
            if goal_handle.accepted:
                # Cancel the goal
                goal_handle.cancel_goal_async()
                self.get_logger().info('Cancel request sent')
            else:
                self.get_logger().info('No active goal to cancel')
        else:
            self.get_logger().info('No goal has been sent yet')


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    if len(sys.argv) < 3:
        print("Usage: elevator_button_client.py <floor_number> <apriltag_frame>")
        print("Example: elevator_button_client.py 2 tag_36h11_123")
        rclpy.shutdown()
        return
    
    try:
        floor_number = int(sys.argv[1])
        apriltag_frame = sys.argv[2]
    except ValueError:
        print("Error: Floor number must be an integer")
        rclpy.shutdown()
        return
    
    client_node = ElevatorButtonPushClient()
    
    # Use MultiThreadedExecutor for handling callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(client_node)
    
    # Send the goal
    client_node.send_goal(floor_number, apriltag_frame)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        client_node.get_logger().info('Keyboard interrupt received, canceling goal')
        client_node.cancel_goal()
        executor.spin_once()  # Process any remaining callbacks
    finally:
        client_node.destroy_node()


if __name__ == '__main__':
    main()