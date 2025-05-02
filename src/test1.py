#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Quaternion
from roarm_moveit.srv import MovePointCmd
from go2_interfaces.action import PushElevatorButton
import time
import numpy as np

class ElevatorButtonPushActionServer(Node):
    # Offset dictionary for floor buttons
    ELEVATOR_BUTTON_OFFSETS = {  # offset from z-axis
        101: 0.1,  # call elevator (up)
        102: -0.1, # call elevator (down)
        -1: -0.1,
        0: 0.0,
        1: 0.1,
        2: 0.2,
        3: 0.3,
        4: 0.4,
    }

    # Camera to Robot Frame Transformation
    ROBOT_TO_CAMERA_TRANSLATION = np.array([0.18, 0.033, 0.0])  # in meters

    ROBOT_TO_CAMERA_ROTATION = np.array([
        [0, 0, 1],
        [-1, 0, 0],
        [0, -1, 0]
    ])
    
    def __init__(self):
        super().__init__('elevator_button_push_action_server')
    
        callback_group = ReentrantCallbackGroup()
        
        self._action_server = ActionServer(
            self,
            PushElevatorButton,
            'push_elevator_button',
            self.execute_callback,
            callback_group=callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.apriltag_pose = None  # Store both position and orientation
        self._goal_handle = None
        self._goal_canceled = False
        self._target_apriltag_frame = None
        self._robot_home_pose = (0.4, 0.0, 0.2)
        
        # AprilTag pose subscriber
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.apriltag_pose_callback,
            10,
            callback_group=callback_group
        )

        self.client = self.create_client(
            MovePointCmd, 
            'move_point_cmd',
            callback_group=callback_group
        )
        self.get_logger().info('Waiting for move_point_cmd service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.get_logger().info('Elevator Button Push Action Server is ready')

    def apriltag_pose_callback(self, msg):
        """Callback to process TF messages and extract AprilTag pose (position and orientation)"""
        for transform in msg.transforms:
            if self._target_apriltag_frame and transform.child_frame_id == self._target_apriltag_frame:
                # Extract position
                position = np.array([
                    transform.transform.translation.x, 
                    transform.transform.translation.y, 
                    transform.transform.translation.z
                ])
                # Extract orientation (quaternion)
                orientation = transform.transform.rotation
                self.apriltag_pose = (position, orientation)

    def goal_callback(self, goal_request):
        """Called when a new goal is received"""
        self.get_logger().info(f'Received goal request: floor {goal_request.floor_number}')
        
        # Check if the requested floor is valid
        if goal_request.floor_number not in self.ELEVATOR_BUTTON_OFFSETS:
            self.get_logger().error(f'Invalid floor number: {goal_request.floor_number}')
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Called when a goal is being canceled"""
        self.get_logger().info('Received cancel request')
        self._goal_canceled = True
        return CancelResponse.ACCEPT

    def quaternion_to_rotation_matrix(self, quat):
        """Convert quaternion to rotation matrix"""
        q = [quat.x, quat.y, quat.z, quat.w]
        qx, qy, qz, qw = q
        return np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])

    async def execute_callback(self, goal_handle):
        """Execute the goal"""
        self.get_logger().info('Executing goal...')
        self._goal_handle = goal_handle
        self._goal_canceled = False
        
        # Create and populate result
        result = PushElevatorButton.Result()
        
        # Extract floor number and apriltag_frame
        self._target_apriltag_frame = goal_handle.request.apriltag_frame
        floor_number = goal_handle.request.floor_number
        
        if floor_number not in self.ELEVATOR_BUTTON_OFFSETS:
            self.get_logger().error(f'Invalid floor number: {floor_number}')
            result.success = False
            goal_handle.abort()
            return result

        
        feedback_msg = PushElevatorButton.Feedback()
        feedback_msg.current_state = 'WAITING_FOR_APRILTAG'
        goal_handle.publish_feedback(feedback_msg)

        
        start_time = self.get_clock().now()
        while self.apriltag_pose is None:
            if self._goal_canceled:
                goal_handle.canceled()
                result.success = False
                return result
                
            current_time = self.get_clock().now()
            if (current_time - start_time).nanoseconds / 1e9 > 10.0:
                self.get_logger().error('Timeout waiting for AprilTag detection')
                result.success = False
                goal_handle.abort()
                return result
                
            
            time.sleep(0.1)

        feedback_msg.current_state = 'PLANNING_MOVEMENT'
        goal_handle.publish_feedback(feedback_msg)
        try:
            apriltag_position, apriltag_orientation = self.apriltag_pose
            robot_to_apriltag_coord = (
                self.ROBOT_TO_CAMERA_TRANSLATION + 
                np.dot(self.ROBOT_TO_CAMERA_ROTATION, apriltag_position)
            )
            R = self.quaternion_to_rotation_matrix(apriltag_orientation)
            normal_vector = np.dot(self.ROBOT_TO_CAMERA_ROTATION, R[:, 2])
            normal_vector = normal_vector / np.linalg.norm(normal_vector)  
            x, y, z = robot_to_apriltag_coord
            button_offset = self.ELEVATOR_BUTTON_OFFSETS[floor_number]

            y_axis = np.dot(self.ROBOT_TO_CAMERA_ROTATION, R[:, 1])  
            button_position = robot_to_apriltag_coord + button_offset * y_axis
            
            approach_distance = 0.1  # 10 cm
            push_distance = 0.03     # 3 cm to press the button
            
            # Define push sequence: approach, push, retract
            approach_pos = button_position - approach_distance * normal_vector
            push_pos = button_position
            retract_pos = approach_pos
            
            points = [
                (approach_pos[0], approach_pos[1], approach_pos[2]),  # Approach
                (push_pos[0], push_pos[1], push_pos[2]),             # Push
                (retract_pos[0], retract_pos[1], retract_pos[2])     # Retract
            ]
            
            # Execute movement sequence
            for idx, (px, py, pz) in enumerate(points):
                if self._goal_canceled:
                    goal_handle.canceled()
                    result.success = False
                    return result
                    
                # Update feedback with current movement phase
                if idx == 0:
                    current_state = 'APPROACHING_BUTTON'
                elif idx == 1:
                    current_state = 'PRESSING_BUTTON'
                else:
                    current_state = 'RETRACTING_ARM'
                    
                feedback_msg.current_state = current_state
                goal_handle.publish_feedback(feedback_msg)
                
                self.get_logger().info(f'Moving to point {idx+1}: ({px:.3f}, {py:.3f}, {pz:.3f})')
                
                request = MovePointCmd.Request()
                request.x = px
                request.y = py
                request.z = pz
                
                response = await self.client.call_async(request)
                
                if not response.success:
                    self.get_logger().error(f'Movement failed: {response.message}')
                    result.success = False
                    goal_handle.abort()
                    return result
                
                # Add a small delay between movements
                time.sleep(0.5)

            # Return to home position
            feedback_msg.current_state = 'RETURNING_TO_HOME'
            goal_handle.publish_feedback(feedback_msg)

            home_x, home_y, home_z = self._robot_home_pose
            self.get_logger().info(f'Returning to home position: ({home_x:.3f}, {home_y:.3f}, {home_z:.3f})')

            request = MovePointCmd.Request()
            request.x = home_x
            request.y = home_y
            request.z = home_z

            response = await self.client.call_async(request)

            if not response.success:
                self.get_logger().error(f'Movement failed: {response.message}')
                result.success = False
                goal_handle.abort()
                return result

            self.get_logger().info(f'Successfully pushed button for floor {floor_number}')
            result.success = True
            goal_handle.succeed()
            
        except Exception as e:
            self.get_logger().error(f'Error during button push execution: {str(e)}')
            result.success = False
            goal_handle.abort()
            
        return result

def main(args=None):
    rclpy.init(args=args)
    
    elevator_action_server = ElevatorButtonPushActionServer()
    
    # Use MultiThreadedExecutor to handle callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(elevator_action_server)
    
    try:
        elevator_action_server.get_logger().info('Starting elevator button push action server...')
        executor.spin()
    except KeyboardInterrupt:
        elevator_action_server.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        executor.shutdown()
        elevator_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()