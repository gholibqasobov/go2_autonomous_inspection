import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class CameraServoPublisher(Node):

    def __init__(self, x:str, y:str):
        super().__init__('camera_servo_publisher')
        self.publisher_ = self.create_publisher(String, '/servo_move', 10)
        timer_period = 1.5  # seconds
        self.timer = self.create_timer(timer_period, self.servo_callback)

        self.x = x
        self.y = y

    def servo_callback(self):
        msg = String()
        msg.data = f"{self.x} {self.y}"

        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {self.x}, {self.y}")


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 3:
        print("Usage: ros2 run <package> <executable> <x> <y>")
        return

    x, y = str(sys.argv[1]), str(sys.argv[2])
   

    minimal_publisher = CameraServoPublisher(x, y)

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


'''

[INFO] [1752313503.557251719] [camera_servo_node]: Sent to serial: std_msgs.msg.String(data='1500 1500')
[INFO] [1752313468.483084429] [camera_servo_node]: Sent to serial: std_msgs.msg.String(data='{"data": "1500 1500"}')

[INFO] [1752313629.038020907] [camera_servo_node]: Sent to serial: std_msgs.msg.String(data='2000 2000')
'''