import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class TwistPublisher(Node):
    def __init__(self, x, y, theta):
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel_robot', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.x = x
        self.y = y
        self.theta = theta

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = self.x
        msg.twist.linear.y = self.y
        msg.twist.angular.z = self.theta
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: x={self.x}, y={self.y}, theta={self.theta}')


def main(args=None):
    rclpy.init(args=args)

    # Expecting 3 numeric arguments: x y theta
    if len(sys.argv) < 4:
        print("Usage: ros2 run <package_name> <node_name> <x> <y> <theta>")
        return

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    theta = float(sys.argv[3])

    node = TwistPublisher(x, y, theta)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
