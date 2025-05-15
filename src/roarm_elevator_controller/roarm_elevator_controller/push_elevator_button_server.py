import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_msgs.msg import TFMessage
from roarm_moveit.srv import MovePointCmd
from go2_interfaces.action import PushElevatorButton
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import math


class ElevatorButtonPushActionServer(Node):
    # Offset dictionary for floor buttons
    ELEVATOR_BUTTON_OFFSETS = {  # offset from z-axis
        101: 0.4255,  # call elevator (up)
        102: 0.477, # call elevator (down)
        -1: -0.1,
        0: -0.1,
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
        
        # Use a reentrant callback group to allow concurrent processing
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
        
        
        self.apriltag_coord = None
        self._goal_handle = None
        self._goal_canceled = False
        self._target_apriltag_frame = None
        self._robot_home_pose = (0.3, 0.0, 0.3)
        self._roll = 0.0
        self._x_offset = 0.05
        self._y_offset = 0.0
        self._z_offset = 0.0
        self._panel_offset = 0.04
        
        # AprilTag pose subscriber
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.apriltag_pose_callback,
            10,
            callback_group=callback_group
        )

        # Robot Move Point Client
        self.client = self.create_client(
            MovePointCmd, 
            'move_point_cmd',
            callback_group=callback_group
        )

        # Wait for service to be available
        self.get_logger().info('Waiting for move_point_cmd service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.get_logger().info('Elevator Button Push Action Server is ready')

    def __quaternion_to_euler(self, qx, qy, qz, qw):
        # Create a rotation object from quaternion
        r = R.from_quat([qx, qy, qz, qw])
        
        # Convert to Euler angles (roll, pitch, yaw) in radians
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        
        print(f"Roll: {roll:.4f} radians, Pitch: {pitch:.4f} radians, Yaw: {yaw:.4f} radians")
        return roll, pitch, yaw

    def apriltag_pose_callback(self, msg):
        """Callback to process TF messages and extract AprilTag pose"""
        for transform in msg.transforms:
            if self._target_apriltag_frame and transform.child_frame_id == self._target_apriltag_frame:
                self.apriltag_coord = np.array([
                    transform.transform.translation.x, 
                    transform.transform.translation.y, 
                    transform.transform.translation.z
                ])

                qx = transform.transform.rotation.x
                qy = transform.transform.rotation.y
                qz = transform.transform.rotation.z
                qw = transform.transform.rotation.w

                self._roll, pitch, yaw = self.__quaternion_to_euler(qx, qy, qz, qw)


                
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

        # Set initial feedback
        feedback_msg = PushElevatorButton.Feedback()
        feedback_msg.current_state = 'WAITING_FOR_APRILTAG'
        goal_handle.publish_feedback(feedback_msg)

        # Wait for AprilTag detection with timeout
        start_time = self.get_clock().now()
        while self.apriltag_coord is None:
            if self._goal_canceled:
                goal_handle.canceled()
                result.success = False
                return result
                
            # Check timeout (10 seconds)
            current_time = self.get_clock().now()
            if (current_time - start_time).nanoseconds / 1e9 > 10.0:
                self.get_logger().error('Timeout waiting for AprilTag detection')
                result.success = False
                goal_handle.abort()
                return result
                
            # Sleep briefly to avoid burning CPU
            time.sleep(0.1)

        
        # Update feedback
        feedback_msg.current_state = 'PLANNING_MOVEMENT'
        goal_handle.publish_feedback(feedback_msg)
        
        # Calculate robot coordinates from camera coordinates
        try:
            robot_to_apriltag_coord = (
                self.ROBOT_TO_CAMERA_TRANSLATION + 
                np.dot(self.ROBOT_TO_CAMERA_ROTATION, self.apriltag_coord)
            )
            
            x, y, z = robot_to_apriltag_coord
            button_offset = self.ELEVATOR_BUTTON_OFFSETS[floor_number]
            button_press_offset = 0.02  # Approach offset
            # self._y_offset = np.sin(self._roll) * 0.03
            self._y_offset = 0.0055
            # Define push sequence: approach, push, retract
            points = [
                (x - self._panel_offset - self._x_offset, y - self._y_offset, z + button_offset),  # Approach
                (x + button_press_offset, y - self._y_offset, z + button_offset - 0.035),             # Push
                (x - self._panel_offset - self._x_offset, y - self._y_offset, z + button_offset)   # Retract
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
                time.sleep(0.7)

            # return to home position
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