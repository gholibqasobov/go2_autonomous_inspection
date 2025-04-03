import sys
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from roarm_moveit.srv import MovePointCmd
import numpy as np

class RoarmElevatorController(Node):
    def __init__(self):
        super().__init__('roarm_elevator_button_controller')

        self.apriltag_coord = None

        # AprilTag pose subscriber
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.apriltag_pose_callback,
            10
        )

        # Robot Move Point Client
        self.client = self.create_client(MovePointCmd, 'move_point_cmd')

        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Service not available, waiting again...')

    def apriltag_pose_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'elevator_docking':
                # self.get_logger().info(f"Transform received for frame: {transform.child_frame_id}")
                # self.get_logger().info(f"Translation: x = {transform.transform.translation.x}, "
                #                        f"y = {transform.transform.translation.y}, "
                #                        f"z = {transform.transform.translation.z}")
                # self.get_logger().info(f"Rotation (Quaternion): x = {transform.transform.rotation.x}, "
                #                        f"y = {transform.transform.rotation.y}, "
                #                        f"z = {transform.transform.rotation.z}, "
                #                        f"w = {transform.transform.rotation.w}")
                self.apriltag_coord = np.array([transform.transform.translation.x, 
                                                transform.transform.translation.y, 
                                                transform.transform.translation.z])
        
    def get_apriltag_coord(self):
        return self.apriltag_coord
           
    def roarm_move_point_request(self, x, y, z):
        request = MovePointCmd.Request()
        request.x = x
        request.y = y
        request.z = z

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    

import time 

def main(args=None):
    rclpy.init(args=args)

    node = RoarmElevatorController()
    time.sleep(3)
    
    rclpy.spin_once(node, timeout_sec=0.5)
    apriltag_coord = node.get_apriltag_coord()
    node.get_logger().info(f'AprilTag Coordinates: {apriltag_coord}')
    
    # apriltag_coord = node.get_apriltag_coord()

    # print(apriltag_coord)
    

    robot_to_camera_coord = np.array([0.14, 0.033, 0.0]) # in meters
    
    robot_to_camera_rot_mtx = np.array([
        [0, 0, 1],
        [-1, 0, 0],
        [0, -1, 0]
    ])
    
    robot_to_apriltag_coord = robot_to_camera_coord + np.dot(robot_to_camera_rot_mtx, apriltag_coord)

    x, y, z = robot_to_apriltag_coord
    
    
    response = node.roarm_move_point_request(x, y, z)


    if response.success:
        node.get_logger().info(f'Success: {response.message}')
    else:
        node.get_logger().warn(f'Failed: {response.message}')


    # rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
