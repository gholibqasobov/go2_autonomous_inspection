import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Joy
import yaml
import time

from go2_interfaces.action import WaypointFollower  # Your custom action


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

        self.goal_pub = self.create_publisher(PointStamped, 'way_point', 10)
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)

        # Publisher for Servo
        self.servo_publisher_ = self.create_publisher(String, '/servo_move', 10)



        self.goals = []
        self.goal_index = 0
        self._goal_handle = None
        self._goal_canceled = False

        self.home_pose = '{ "data":"2000 2000"}'
        self.servo_angles = [self.home_pose, '{ "data":"2000 3000"}', '{ "data":"2000 1000"}', self.home_pose]

    def goal_callback(self, goal_request):
        self.get_logger().info(f"Received goal request: {goal_request.waypoint_file}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        self._goal_canceled = True
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self._goal_handle = goal_handle
        self._goal_canceled = False

        filename = goal_handle.request.waypoint_file
        result = WaypointFollower.Result()

        # Relative Path: src/go2_waypoint_nav/navigation_path/test_waypoint.yaml

        try:
            with open(f'/home/dog/go2_autonomous_inspection/src/go2_waypoint_nav/navigation_path/{filename}', 'r') as file:
                data = yaml.safe_load(file)
                self.goals = data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f"Failed to load {filename}: {e}")
            result.success = False
            goal_handle.abort()
            return result

        self.goal_index = 0
        self.send_auto_mode_joy()
        # await rclpy.task.sleep(2.0)  # Wait for auto mode to engage
        time.sleep(2.0)

        for goal_data in self.goals:
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled")
                goal_handle.canceled()
                result.success = False
                return result

            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = goal_data['header']['frame_id']
            msg.point.x = goal_data['pose']['position']['x']
            msg.point.y = goal_data['pose']['position']['y']
            msg.point.z = goal_data['pose']['position']['z']

            self.goal_pub.publish(msg)
            self.get_logger().info(f"Published goal {self.goal_index + 1}/{len(self.goals)}: {msg}")

            feedback = WaypointFollower.Feedback()
            feedback.current_position = f"Sent goal {self.goal_index + 1}"
            goal_handle.publish_feedback(feedback)

            self.goal_index += 1
            # await rclpy.task.sleep(7.0)
            time.sleep(15.0)
            servo_msg = String()
            for direction in self.servo_angles:
                start_time = time.time()
                while time.time() - start_time < 4.0:
                    servo_msg.data = direction
                    self.servo_publisher_.publish(servo_msg)
                    self.get_logger().info(f"Publishing: {direction}")
                    time.sleep(0.2)
                time.sleep(5)
            
            # time.sleep(2.0)

        self.get_logger().info("All goals sent successfully")
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
