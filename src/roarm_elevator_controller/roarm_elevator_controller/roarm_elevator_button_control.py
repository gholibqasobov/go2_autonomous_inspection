import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class RoarmElevatorController(Node):
    def __init__(self):
        super().__init__('apriltag_pose_subscriber')
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.apriltag_pose_callback,
            10
        )

    def apriltag_pose_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'elevator_docking':
                self.get_logger().info(f"Received transform for frame: {transform.child_frame_id}")
                self.get_logger().info(f"Translation: x = {transform.transform.translation.x}, "
                                       f"y = {transform.transform.translation.y}, "
                                       f"z = {transform.transform.translation.z}")
                self.get_logger().info(f"Rotation (Quaternion): x = {transform.transform.rotation.x}, "
                                       f"y = {transform.transform.rotation.y}, "
                                       f"z = {transform.transform.rotation.z}, "
                                       f"w = {transform.transform.rotation.w}")

def main(args=None):
    rclpy.init(args=args)
    node = RoarmElevatorController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
