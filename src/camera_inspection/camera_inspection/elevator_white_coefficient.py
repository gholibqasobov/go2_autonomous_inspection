import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
import time

# ----------------------------------------------------------------------
# white‐detection parameters
WHITE_SAT_MAX = 50    # allow only desaturated (near‐gray/white) pixels
WHITE_VAL_MIN = 180   # require brightness
# ----------------------------------------------------------------------


class ElevatorWhiteCoefficient(Node):
    def __init__(self):
        super().__init__('elevator_white_coefficient')

        self.bridge = CvBridge()
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.process_image,
            10
        )

        # State variables
        self.last_log_t = 0.0  # Time of last coefficient log
        self.log_interval = 0.5  # Log coefficient every 0.5 seconds
        
        self.get_logger().info('Elevator white coefficient detector started')

    def white_mask(self, image, sat_max=WHITE_SAT_MAX, val_min=WHITE_VAL_MIN):
        """binary mask of 'white enough' pixels in image"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # allow any hue, sat <= sat_max, val >= val_min
        lower = np.array([0,      0,       val_min])
        upper = np.array([180, sat_max,     255])
        mask  = cv2.inRange(hsv, lower, upper)

        # clean up
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        return mask

    def process_image(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Use the entire frame for detection
            # Get white mask
            white_m = self.white_mask(frame)
            white_coef = white_m.mean() / 255.0

            # Log the white coefficient value regularly
            now = time.time()
            if now - self.last_log_t >= self.log_interval:
                self.get_logger().info(f"White coefficient: {white_coef:.6f}")
                self.last_log_t = now

            # Overlay the white coefficient
            h, w = frame.shape[:2]
            text = f"White coef: {white_coef:.6f}"
            cv2.putText(frame, text, (10, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)

            # Display the processed frame
            try:
                cv2.imshow("White Coefficient Detector", frame)
                cv2.imshow("White Mask", white_m)
                cv2.waitKey(1)
            except Exception as e:
                # Ignore display errors - this may run headless
                pass
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    detector = ElevatorWhiteCoefficient()
    rclpy.spin(detector)
    
    # Cleanup
    cv2.destroyAllWindows()
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()