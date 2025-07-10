import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Joy
import yaml
import sys
import time
from go2_interfaces.action import WaypointFollower
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor


class WaypointFollowerServer(Node):
    
    def __init__(self):
        super().__init__('waypoint_follower_server')

        callback_group = ReentrantCallbackGroup()

        self._waypoint_follower_server = ActionServer(
            self,
            WaypointFollower,
            'waypoint_follower',
            self.execute_callback,
            callback_group=callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.goals = None
        self._goal_handle = None
        self._goal_canceled = None




        # Robot Waypoint Publisher Topic
        self.goal_pub = self.create_publisher(PointStamped, 'way_point', 10)
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)





        def goal_callback(self, goal_request):
            self.get_logger().info(f'Received goal request - waypoint_file: {goal_request.waypoint_file}')

            return GoalResponse.ACCEPT

        def cancel_callback(self, goal_handle):
            self.get_logger().info(f'Received cancel request')
            self._goal_canceled = True
            return CancelResponse.ACCEPT

        async def execute_callback(self, goal_handle):
            self._goal_hanle = goal_handle
            self._goal_canceled = False

            # Load waypoints from YAML
            try:
                with open('src/go2_waypoint_nav/navigation_path/' + goal_handle.request.waypoint_file, 'r') as file:
                    data = yaml.safe_load(file)
                    # save the goals
                    self.goals = data.get('waypoints', [])
            except Exception as e:
                self.get_logger().error(f"Failed to load {goal_handle.request.waypoint_file}: {e}")
                rclpy.shutdown()
                sys.exit(1)

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

        def send_auto_mode_joy(self):
            joy_msg = Joy()
            joy_msg.header.stamp = self.get_clock().now().to_msg()
            joy_msg.header.frame_id = "teleop_panel"
            joy_msg.axes = [0.0] * 8
            joy_msg.buttons = [0] * 11
            joy_msg.axes[2] = -1.0  # index 5 sets AUTO mode
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

    waypoint_follower_server = WaypointFollowerServer()

    # Use MultiThreadedExecutor to handle callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_follower_server)

    try:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        waypoint_follower_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
