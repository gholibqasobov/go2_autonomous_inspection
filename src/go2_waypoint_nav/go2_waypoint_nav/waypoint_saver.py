import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPoseSubscriber(Node):

    def __init__(self):
        super().__init__('goal_pose_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(
            f'I heard goal pose - Position: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}, '
            f'Orientation: x={msg.pose.orientation.x}, y={msg.pose.orientation.y}, '
            f'z={msg.pose.orientation.z}, w={msg.pose.orientation.w}'
        )


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = GoalPoseSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()