import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Joy
import time

from go2_interfaces.action import WaypointFollower 

from tf2_msgs.msg import TFMessage
import numpy as np

import asyncio

class WaypointFollowerServer(Node):

    def __init__(self):
        super().__init__('waypoint_follower_server')

        self.callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            WaypointFollower,
            'waypoint_follower',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        # TF subscriber
        self.tf_subscriber = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )

        # Store current and last norm
        self.normalized = None
        self.last_value = None

        self.goal_reached = False
        self._goal_reached_event = asyncio.Event()

        # Timer to check every 1.5 seconds
        self.timer = self.create_timer(1.5, self.check_reached_goal)


        self.goal_pub = self.create_publisher(PointStamped, 'way_point', 10)
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)

        self._goal_handle = None
        self._goal_canceled = False

    """================================================================"""
    def tf_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            t = transform.transform.translation
            r = transform.transform.rotation

            self.normalized = self.norm(t.x, t.y, t.z, r.z)


    def norm(self, x, y, z, rz):
        return np.linalg.norm([x, y, z, rz])

    def check_reached_goal(self):
        """Check every 1.5 seconds if the robot has stopped moving."""
        if self.normalized is None:
            return  

        if self.last_value is None:
            self.last_value = self.normalized
            return

        if np.isclose(self.last_value, self.normalized, atol=1e-6):
            if not self.goal_reached:
                self.get_logger().info("Robot reached goal position.")
                self.goal_reached = True
                # Signal the event to wake up the waiting coroutine
                self._goal_reached_event.set()
        else:
            self.last_value = self.normalized
            if self.goal_reached:
                self.goal_reached = False
                self._goal_reached_event.clear()
    """================================================================"""
            
    def goal_callback(self, goal_request):
        self.get_logger().info(f"Received goal request: {goal_request.waypoint}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        self._goal_canceled = True
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self._goal_handle = goal_handle
        self._goal_canceled = False
        self.goal_reached = False

        result = WaypointFollower.Result()

        self.send_auto_mode_joy()
        time.sleep(2.0)  # blocking sleep

        if goal_handle.is_cancel_requested:
            self.get_logger().info("Goal canceled")
            goal_handle.canceled()
            result.success = False
            return result

        waypoint = goal_handle.request.waypoint
        self.goal_pub.publish(waypoint)
        self.get_logger().info(f"Published goal {waypoint}")

        feedback = WaypointFollower.Feedback()
        feedback.current_position = f"Moving to goal: {waypoint}"
        goal_handle.publish_feedback(feedback)

        while not self.goal_reached:
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled while waiting")
                goal_handle.canceled()
                result.success = False
                return result
            rclpy.spin_once(self, timeout_sec=0.1)  # Process incoming TF msgs, timer callbacks, etc.

        self.get_logger().info("Waypoint execution complete")
        result.success = True
        goal_handle.succeed()
        return result

    def send_auto_mode_joy(self):
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.header.frame_id = "teleop_panel"
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 11
        joy_msg.axes[2] = -1.0  # Auto mode activation
        joy_msg.axes[4] = 0.3
        self.joy_pub.publish(joy_msg)


def main(args=None):
    rclpy.init(args=args)
    server = WaypointFollowerServer()

    executor = MultiThreadedExecutor()
    executor.add_node(server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
