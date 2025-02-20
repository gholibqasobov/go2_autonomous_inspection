import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GoalPosePublisher(Node):

    def __init__(self, filename):
        super().__init__('goal_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'way_point', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = PoseStamped()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    goal_pose_subscriber = GoalPosePublisher()

    rclpy.spin(goal_pose_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    goal_pose_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()