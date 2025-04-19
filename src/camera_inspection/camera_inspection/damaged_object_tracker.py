import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DamagedObjectTracker(Node):
    

    def __init__(self):
        super().__init__('damaged_object_tracking')
       
        self.robot_odom_sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.save_robot_odom,
            10
        )
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.save_damaged_image,
            10
        )
        self.robot_odom_sub  # prevent unused variable warning
        self.camera_sub

        self.cv_bridge = CvBridge()

    
    def save_robot_odom(self, msg):
        """Callback function that gets called when a new TF message is received."""
        # self.get_logger().info(f'Received TF message with {msg.transforms}')
        x, y, z = msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y, msg.transforms[0].transform.translation.z 
        self.get_logger().info(f"x is {x}, y is {y}, z is {z}")
        return x, y, z

    def save_damaged_image(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Display image dimensions and data type
            self.get_logger().info(f'  OpenCV image shape: {cv_image.shape}, dtype: {cv_image.dtype}')
            
            # Display the image
            cv2.imshow('Camera Image', cv_image)
            cv2.waitKey(1)  # Wait 1ms, allows the window to update
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = DamagedObjectTracker()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()