import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.pub_speed = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.move_forward()

    def move_forward(self):
        self.get_logger().info('Moving forward for 2 seconds...')
        cmd_vel = TwistStamped()
        cmd_vel.header.frame_id = 'vehicle'
        cmd_vel.twist.linear.x = 0.0  # Adjust speed as needed
        cmd_vel.twist.angular.z = 0.2

        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        rate = self.create_rate(10)
        
        while self.get_clock().now().seconds_nanoseconds()[0] - start_time < 2.0:
            cmd_vel.header.stamp = self.get_clock().now().to_msg()
            self.pub_speed.publish(cmd_vel)
            rate.sleep()

        # Stop the robot
        self.get_logger().info('Stopping robot...')
        cmd_vel.twist.linear.x = 0.0
        self.pub_speed.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
