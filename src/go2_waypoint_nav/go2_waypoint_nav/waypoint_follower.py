import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Joy
import yaml
import sys
import time

class GoalPosePublisher(Node):

    def __init__(self, filename):
        super().__init__('goal_pose_publisher')

        # Publishers
        self.goal_pub = self.create_publisher(PointStamped, 'way_point', 10)
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)

        # Load waypoints from YAML
        try:
            with open('src/go2_waypoint_nav/navigation_path/' + filename, 'r') as file:
                data = yaml.safe_load(file)
                self.goals = data.get('waypoints', [])
        except Exception as e:
            self.get_logger().error(f"Failed to load {filename}: {e}")
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

    filename = 'test_waypoint'
    if not filename.endswith('.yaml'):
        filename += '.yaml'

    goal_pose_publisher = GoalPosePublisher(filename)

    try:
        rclpy.spin(goal_pose_publisher)
    except KeyboardInterrupt:
        goal_pose_publisher.get_logger().info("Keyboard interrupt. Shutting down...")
    finally:
        goal_pose_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()  