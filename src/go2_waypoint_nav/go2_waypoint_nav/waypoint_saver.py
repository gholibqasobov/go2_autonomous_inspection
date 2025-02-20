import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import json
import sys

class GoalPoseSubscriber(Node):

    def __init__(self, filename, num_points):
        super().__init__('goal_pose_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.listener_callback,
            10)

        self.filename = filename # file name defined by user
        self.num_points = num_points
        self.goals = []
        self.z = 0.3659999966621399
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        goal_data = {
            'header': {
                'frame_id': msg.header.frame_id
            },
            'pose': {
                'position': {
                    'x': msg.pose.position.x,
                    'y': msg.pose.position.y,
                    'z': self.z
                },
                'orientation': {
                    'x': msg.pose.orientation.x,
                    'y': msg.pose.orientation.y,
                    'z': msg.pose.orientation.z,
                    'w': msg.pose.orientation.w
                }
            }
        }

        # add goal to the list
        self.goals.append(goal_data)

        self.get_logger().info("I heard: \n" + json.dumps(goal_data, indent=2))


        if len(self.goals)  == self.num_points:
            self.save_goals_to_yaml()


        

    def save_goals_to_yaml(self):
        data = {'waypoints': self.goals}
        
        with open('src/go2_waypoint_nav/navigation_path/' + self.filename, 'w') as file:
            yaml.dump(data, file ,default_flow_style=False)
        
        self.get_logger().info(f"Goals saved to {self.filename}")
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)


    # filename = input("Enter the name of .yaml file: ")
    filename = 'test_waypoint'
    if not filename.endswith('.yaml'):
        filename += '.yaml'


    num_points = int(input('Enter number of goal points: '))


    goal_pose_subscriber = GoalPoseSubscriber(filename, num_points)

    try:
        rclpy.spin(goal_pose_subscriber)
    except KeyboardInterrupt:
        goal_pose_subscriber.get_logger().info("Shutting down.")
    finally:
        goal_pose_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()