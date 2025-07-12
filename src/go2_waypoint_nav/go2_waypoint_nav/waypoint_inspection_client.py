import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml
from geometry_msgs.msg import PointStamped
from go2_interfaces.action import WaypointFollower
import os
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import time

class WaypointFollowerClient(Node):

    def __init__(self):
        super().__init__('waypoint_follower_client')
        self.callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self, 
            WaypointFollower, 
            'waypoint_follower',
            callback_group=self.callback_group
        )
        self.current_goal_handle = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.timer = None

    def load_waypoints_from_yaml(self, yaml_file_path):
        """Load waypoints from YAML file"""
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

    def start_waypoint_sequence(self, interval_sec=15):
        """Start sending waypoints at specified interval"""
        if not self.waypoints:
            self.get_logger().error("No waypoints loaded")
            return

        self.get_logger().info(f"Starting waypoint sequence with {interval_sec} second interval")
        self.current_waypoint_index = 0
        self._send_next_waypoint()

    def _send_next_waypoint(self):
        """Send the next waypoint in the sequence"""
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints completed!")
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        position = waypoint['pose']['position']
        
        self.get_logger().info(f"Sending waypoint {self.current_waypoint_index + 1}: {position}")
        self.send_goal(position['x'], position['y'], position['z'])
        self.current_waypoint_index += 1

    def send_goal(self, x, y, z):
        """Send a waypoint goal to the action server"""
        # Create a PointStamped message for the waypoint
        waypoint = PointStamped()
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.header.frame_id = 'map'  # Adjust frame_id as needed
        waypoint.point.x = x
        waypoint.point.y = y
        waypoint.point.z = z

        # Create goal message
        goal_msg = WaypointFollower.Goal()
        goal_msg.waypoint = waypoint

        # Wait for action server
        self._action_client.wait_for_server()

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the response from the action server"""
        self.current_goal_handle = future.result()
        if not self.current_goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = self.current_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the result from the action server"""
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}')
        
        # Schedule next waypoint after 15 seconds
        self.timer = threading.Timer(15.0, self._send_next_waypoint)
        self.timer.start()

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.current_position}')

    def destroy(self):
        """Cleanup before destruction"""
        if self.timer is not None:
            self.timer.cancel()
        if self.current_goal_handle is not None:
            self.current_goal_handle.cancel_goal_async()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    # Create client node
    action_client = WaypointFollowerClient()
    
    # Load waypoints from YAML file
    yaml_file_path = '/home/qasob/go2_autonomous_inspection/src/go2_waypoint_nav/navigation_path/test_waypoint.yaml'  # Update this path if needed
    if not action_client.load_waypoints_from_yaml(yaml_file_path):
        action_client.destroy()
        rclpy.shutdown()
        return
    
    # Start the waypoint sequence
    action_client.start_waypoint_sequence(interval_sec=15)
    
    # Use MultiThreadedExecutor to handle callbacks properly
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