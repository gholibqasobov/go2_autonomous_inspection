import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import time
from std_msgs.msg import String

from go2_interfaces.action import RotateCameraServo  # Your custom action

class CameraServoActionServer(Node):

    def __init__(self):
        super().__init__('camera_servo_action_server')
        
        # Action server
        self._action_server = ActionServer(
            self,
            RotateCameraServo,
            'camera_servo_action',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Publisher for servo commands
        self.publisher_ = self.create_publisher(String, '/servo_move', 10)

        # Servo positions
        self.home_pose = "2000 2000"
        self.directions = ["2000 3000", "2000 1000", "1500 1000", "1500 2000", "1500 3000", "1000 3000", "1000 2000"]
        
        self._goal_handle = None
        self._goal_canceled = False

    def goal_callback(self, goal_request):
        self.get_logger().info(f"Received goal request: start={goal_request.start}")
        if not goal_request.start:
            self.get_logger().info("Rejected goal - start must be true")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        self._goal_canceled = True
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self._goal_handle = goal_handle
        self._goal_canceled = False

        result = RotateCameraServo.Result()

        if not goal_handle.request.start:
            result.success = False
            goal_handle.abort()
            return result

        # First move to home pose
        self._publish_servo_command(self.home_pose)
        feedback_msg = RotateCameraServo.Feedback()
        feedback_msg.current_direction = "home"
        goal_handle.publish_feedback(feedback_msg)
        self._sleep_with_cancel_check(2.0)

        if self._goal_canceled:
            result.success = False
            goal_handle.canceled()
            return result

        # Move through each direction
        for direction in self.directions:
            self._publish_servo_command(direction)
            feedback_msg.current_direction = direction
            goal_handle.publish_feedback(feedback_msg)
            self._sleep_with_cancel_check(4.0)

            if self._goal_canceled:
                result.success = False
                goal_handle.canceled()
                return result

        # Return to home pose
        self._publish_servo_command(self.home_pose)
        feedback_msg.current_direction = "home"
        goal_handle.publish_feedback(feedback_msg)
        self._sleep_with_cancel_check(4.0)

        # Complete the action
        result.success = True
        goal_handle.succeed()
        return result

    def _publish_servo_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published servo command: {command}")

    def _sleep_with_cancel_check(self, duration):
        """Sleep for duration but check for cancel periodically"""
        sleep_interval = 0.1
        remaining = duration
        while remaining > 0 and not self._goal_canceled:
            time.sleep(min(sleep_interval, remaining))
            remaining -= sleep_interval


def main(args=None):
    rclpy.init(args=args)
    server = CameraServoActionServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()