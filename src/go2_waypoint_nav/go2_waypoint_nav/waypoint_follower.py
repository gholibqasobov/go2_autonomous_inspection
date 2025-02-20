import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import yaml
import sys
import time

class GoalPosePublisher(Node):

    def __init__(self, filename):
        super().__init__('goal_pose_publisher')
        self.publisher_ = self.create_publisher(PointStamped, 'way_point', 10)

        try:
            with open('src/go2_waypoint_nav/navigation_path/' + filename, 'r') as file:
                data = yaml.safe_load(file)
                self.goals = data.get('waypoints', [])

        except Exception as e:
            self.get_logger().error(f"Failed to load {filename}: {e}")
            rclpy.shutdown()
            sys.exit(1)

        self.goal_index = 0
            
        self.timer = self.create_timer(1.0, self.publish_next_goal)

    def publish_next_goal(self):
        if self.goal_index < len(self.goals):
            goal_data = self.goals[self.goal_index]

            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = goal_data['header']['frame_id']
            msg.point.x = goal_data['pose']['position']['x']
            msg.point.y = goal_data['pose']['position']['y']
            msg.point.z = goal_data['pose']['position']['z']

            self.publisher_.publish(msg)
            self.get_logger().info(f"Published Goal {self.goal_index + 1}/{len(self.goals)}: {msg}")

            self.goal_index += 1

            if self.goal_index < len(self.goals):
                # Wait for 10 seconds before publishing the next goal
                self.create_timer(10.0, self.publish_next_goal)
                time.sleep(7)
            else:
                self.get_logger().info("All goal points have been reached. Shutting down...")
                self.destroy_node()
                rclpy.shutdown()
                sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    # filename = input('Enter the name of the .yaml file: ')
    filename = 'test_waypoint'
    if not filename.endswith('.yaml'):
        filename += '.yaml'

    goal_pose_publisher = GoalPosePublisher(filename)

    
    try:
        rclpy.spin(goal_pose_publisher)
    except KeyboardInterrupt:
        goal_pose_publisher.get_logger().info("Shutting down...")
    finally:
        goal_pose_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()