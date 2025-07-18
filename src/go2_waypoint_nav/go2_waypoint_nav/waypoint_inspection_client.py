import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml
from geometry_msgs.msg import PointStamped
from go2_interfaces.action import WaypointFollower, RotateCameraServo
import os
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import time
from std_srvs.srv import SetBool

class WaypointFollowerClient(Node):

    def __init__(self):
        super().__init__('waypoint_follower_client')
        self.callback_group = ReentrantCallbackGroup()
        
        # Waypoint follower action client
        self._waypoint_action_client = ActionClient(
            self, 
            WaypointFollower, 
            'waypoint_follower',
            callback_group=self.callback_group
        )
        
        # Camera servo action client
        self._camera_servo_client = ActionClient(
            self,
            RotateCameraServo,
            'camera_servo_action',
            callback_group=self.callback_group
        )
        
        # Inspection service client
        self._inspection_client = self.create_client(
            SetBool,
            '/damaged_object_tracker_server/start_inspection',
            callback_group=self.callback_group
        )
        
        self.current_goal_handle = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.timer = None

    def load_waypoints_from_yaml(self, yaml_file_path):
        if not os.path.exists(yaml_file_path):
            self.get_logger().error(f"YAML file not found: {yaml_file_path}")
            return False

        with open(yaml_file_path, 'r') as file:
            try:
                data = yaml.safe_load(file)
                self.waypoints = data.get('waypoints', [])
                self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from YAML file")
                return True
            except yaml.YAMLError as e:
                self.get_logger().error(f"Error parsing YAML file: {e}")
                return False

    def start_waypoint_sequence(self, interval_sec=12):
        if not self.waypoints:
            self.get_logger().error("No waypoints loaded")
            return

        self.get_logger().info(f"Starting waypoint sequence with {interval_sec} second interval")
        self.current_waypoint_index = 0
        self._send_next_waypoint()

    def _send_next_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints completed!")
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        position = waypoint['pose']['position']
        
        self.get_logger().info(f"Sending waypoint {self.current_waypoint_index + 1}: {position}")
        self.send_goal(position['x'], position['y'], position['z'])
        self.current_waypoint_index += 1

    def send_goal(self, x, y, z):
        waypoint = PointStamped()
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.header.frame_id = 'map'  
        waypoint.point.x = x
        waypoint.point.y = y
        waypoint.point.z = z

        # goal message
        goal_msg = WaypointFollower.Goal()
        goal_msg.waypoint = waypoint

        # Wait for action server
        self._waypoint_action_client.wait_for_server()

        # Send goal
        self._send_goal_future = self._waypoint_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.waypoint_goal_response_callback)

    def waypoint_goal_response_callback(self, future):
        self.current_goal_handle = future.result()
        if not self.current_goal_handle.accepted:
            self.get_logger().info('Waypoint goal rejected :(')
            return

        self.get_logger().info('Waypoint goal accepted :)')

        self._get_result_future = self.current_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.waypoint_result_callback)

    def waypoint_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Waypoint result: {result.success}')
        
        if result.success:
            # Waypoint reached successfully, trigger inspection and camera servo
            self.start_inspection_and_trigger_servo()
        else:
            # Waypoint failed, move to next waypoint after delay
            self.schedule_next_waypoint()

    def start_inspection_and_trigger_servo(self):
        if not self._inspection_client.service_is_ready():
            self.get_logger().warn("Inspection service not available, skipping inspection")
            self.trigger_camera_servo()
            return
            
        self.get_logger().info("Starting inspection before triggering camera servo")
        request = SetBool.Request()
        request.data = True
        
        future = self._inspection_client.call_async(request)
        future.add_done_callback(self.inspection_start_callback)

    def inspection_start_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Inspection started successfully: {response.message}')
            else:
                self.get_logger().warn(f'Failed to start inspection: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
        
        # Regardless of inspection start result, proceed with servo
        self.trigger_camera_servo()

    def trigger_camera_servo(self):
        self.get_logger().info("Triggering camera servo action")
        
        # Wait for camera servo action server
        self._camera_servo_client.wait_for_server()
        
        #goal message
        goal_msg = RotateCameraServo.Goal()
        goal_msg.start = True
        
        # Send goal
        self._send_servo_goal_future = self._camera_servo_client.send_goal_async(
            goal_msg,
            feedback_callback=self.servo_feedback_callback
        )
        self._send_servo_goal_future.add_done_callback(self.servo_goal_response_callback)

    def servo_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Camera servo goal rejected :(')
            self.stop_inspection_and_schedule_next()
            return

        self.get_logger().info('Camera servo goal accepted :)')

        self._get_servo_result_future = goal_handle.get_result_async()
        self._get_servo_result_future.add_done_callback(self.servo_result_callback)

    def servo_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Camera servo result: {result.success}')
        
        # Stop inspection and schedule next waypoint
        self.stop_inspection_and_schedule_next()

    def stop_inspection_and_schedule_next(self):
        if not self._inspection_client.service_is_ready():
            self.get_logger().warn("Inspection service not available, skipping stop")
            self.schedule_next_waypoint()
            return
            
        self.get_logger().info("Stopping inspection after servo action")
        request = SetBool.Request()
        request.data = False
        
        future = self._inspection_client.call_async(request)
        future.add_done_callback(self.inspection_stop_callback)

    def inspection_stop_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Inspection stopped successfully: {response.message}')
            else:
                self.get_logger().warn(f'Failed to stop inspection: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
        
        # Regardless of inspection stop result, schedule next waypoint
        self.schedule_next_waypoint()

    def schedule_next_waypoint(self):
        # next waypoint after default: 12 seconds
        self.timer = threading.Timer(12.0, self._send_next_waypoint)
        self.timer.start()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received waypoint feedback: {feedback.current_position}')

    def servo_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received servo feedback: {feedback.current_direction}')

    def destroy(self):
        if self.timer is not None:
            self.timer.cancel()
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    action_client = WaypointFollowerClient()
    
    # Load waypoints from YAML file
    yaml_file_path = '/home/dog/go2_autonomous_inspection/src/go2_waypoint_nav/navigation_path/test_waypoint.yaml'  # Update this path if needed
    if not action_client.load_waypoints_from_yaml(yaml_file_path):
        action_client.destroy()
        rclpy.shutdown()
        return
    
    # Start the waypoint sequence
    action_client.start_waypoint_sequence(interval_sec=15)
    
    executor = MultiThreadedExecutor()
    executor.add_node(action_client)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        action_client.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        action_client.destroy()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()