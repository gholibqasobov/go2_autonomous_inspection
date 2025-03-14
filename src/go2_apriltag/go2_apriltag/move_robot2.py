import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class RotateRobot(Node):
    def __init__(self):
        super().__init__('rotate_robot')
        self.pub_cmd_vel = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.running_time = 0.0
        self.timer = self.create_timer(0.1, self.rotate)

    def rotate(self):
        if self.running_time < 3.0:
            cmd_vel = TwistStamped()
            cmd_vel.header.stamp = self.get_clock().now().to_msg()
            cmd_vel.twist.angular.z = 0.5  # Rotate at 0.5 rad/s
            self.pub_cmd_vel.publish(cmd_vel)
            self.running_time += 0.1  # Increment elapsed time
        else:
            self.stop_rotation()

    def stop_rotation(self):
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.twist.angular.z = 0.0  # Stop rotation
        self.pub_cmd_vel.publish(cmd_vel)
        self.get_logger().info("Rotation stopped after 3 seconds.")
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = RotateRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

