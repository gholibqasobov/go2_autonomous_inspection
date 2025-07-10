import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Joy
import yaml
import time
import sys

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

        self.goals = []
        self.goal_index = 0
        self._goal_handle = None
        self._goal_canceled = False

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

        try:
            with open(f'src/go2_waypoint_nav/navigation_path/{filename}', 'r') as file:
                data = yaml.safe_load(file)
                self.goals = data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f"Failed to load {filename}: {e}")
            result.success = False
            goal_handle.abort()
            return result

        self.goal_index = 0
        
        # Step control: 0 = send auto mode joy, 1 = wait, 2 = publish goals
        self.step = 0
        self.timer = self.create_timer(1.0, self.run_sequence)

    def run_sequence(self):
        if self.step == 0:
            self.send_auto_mode_joy()
            self.get_logger().info("Sent auto mode Joy message after 3s.")
            self.step += 3

        elif self.step == 3:
            self.get_logger().info("Waiting for auto mode to activate...")
            #time.sleep(2)  # wait 2 seconds
            self.step += 1

        elif self.step == 4:
            self.publish_next_goal()


        # for goal_data in self.goals:
        #     if goal_handle.is_cancel_requested:
        #         self.get_logger().info("Goal canceled")
        #         goal_handle.canceled()
        #         result.success = False
        #         return result

        #     msg = PointStamped()
        #     msg.header.stamp = self.get_clock().now().to_msg()
        #     msg.header.frame_id = goal_data['header']['frame_id']
        #     msg.point.x = goal_data['pose']['position']['x']
        #     msg.point.y = goal_data['pose']['position']['y']
        #     msg.point.z = goal_data['pose']['position']['z']

        #     self.goal_pub.publish(msg)
        #     self.get_logger().info(f"Published goal {self.goal_index + 1}/{len(self.goals)}: {msg}")

        #     feedback = WaypointFollower.Feedback()
        #     feedback.current_position = f"Sent goal {self.goal_index + 1}"
        #     goal_handle.publish_feedback(feedback)

        #     self.goal_index += 1
        #     await rclpy.task.sleep(7.0)

        # self.get_logger().info("All goals sent successfully")
        # result.success = True
        # goal_handle.succeed()
        # return result

    def send_auto_mode_joy(self):
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.header.frame_id = "teleop_panel"
        joy_msg.axes = [0.0] * 8
        joy_msg.buttons = [0] * 11
        joy_msg.axes[2] = -1.0  # Auto mode activation
        joy_msg.axes[4] = 0.3
        self.joy_pub.publish(joy_msg)

    

    def publish_next_goal(self):
        if self.goal_index < len(self.goals):
            goal_data = self.goals[self.goal_index]

            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = goal_data['header']['frame_id']
            msg.point.x = goal_data['pose']['position']['x']
            msg.point.y = goal_data['pose']['position']['y']
            msg.point.z = goal_data['pose']['position']['z']

            self.goal_pub.publish(msg)
            self.get_logger().info(f"Published Goal {self.goal_index + 1}/{len(self.goals)}: {msg}")

            self.goal_index += 1

            if self.goal_index < len(self.goals):
                time.sleep(7)  # wait before next goal
            else:
                self.get_logger().info("All goal points have been sent. Shutting down...")
                self.destroy_node()
                rclpy.shutdown()
                sys.exit(0)


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
