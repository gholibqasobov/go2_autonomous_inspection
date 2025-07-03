import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import json

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

class ServoBridgeNode(Node):
    def __init__(self):
        super().__init__('camera_servo_node')

        # Serial setup
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info(f"Serial connected on {SERIAL_PORT} at {BAUD_RATE} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        # ROS2 Pub/Sub
        self.publisher_ = self.create_publisher(String, 'servo_angles', 10)
        self.subscriber_ = self.create_subscription(String, 'servo_move', self.move_callback, 10)

        # Thread for reading from serial
        self.serial_thread = threading.Thread(target=self.read_from_serial, daemon=True)
        self.serial_thread.start()

    def move_callback(self, msg: String):
        try:
            json_obj = json.loads(msg.data)  # Validate it's JSON
            self.ser.write((json.dumps(json_obj) + '\n').encode())
            self.get_logger().info(f"Sent to serial: {json_obj}")
        except json.JSONDecodeError:
            self.get_logger().error("Received invalid JSON for servo_move")

    def read_from_serial(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode().strip()
                if line:
                    self.get_logger().info(f"Received from serial: {line}")
                    msg = String()
                    msg.data = line
                    self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ServoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

