import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
import time

# ----------------------------------------------------------------------
# configurable parameters
SAT_MIN = 30                  # minimum saturation for red detection
VAL_MIN = 50                  # minimum brightness/value for red detection
HUE_LOW_1, HUE_HIGH_1 = 0, 10  # first range of red hue
HUE_LOW_2, HUE_HIGH_2 = 170, 180  # second range of red hue
RED_DOM_RATIO = 0.40          # R / (R+G+B) must exceed this
# ----------------------------------------------------------------------


class ElevatorRedCoefficient(Node):
    def __init__(self):
        super().__init__('elevator_red_coefficient')

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
        
        self.get_logger().info('Elevator red coefficient detector started')

    def red_mask(self, image, s_min=SAT_MIN, v_min=VAL_MIN):
        """binary mask of 'red enough' pixels in image"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        m1 = cv2.inRange(hsv, (HUE_LOW_1, s_min, v_min),
                             (HUE_HIGH_1, 255, 255))
        m2 = cv2.inRange(hsv, (HUE_LOW_2, s_min, v_min),
                             (HUE_HIGH_2, 255, 255))
        mask = m1 | m2

        # morphology to remove specks / fill holes
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        # enforce "red channel dominates"
        b, g, r = cv2.split(image)
        ratio = r.astype(np.float32) / (r + g + b + 1)
        mask &= (ratio > RED_DOM_RATIO).astype(np.uint8) * 255
        return mask

    def process_image(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Use the entire frame for red detection
            # Get red mask using the improved function
            red_mask = self.red_mask(frame)
            red_ratio = red_mask.mean() / 255.0

            # Log the red coefficient value regularly
            now = time.time()
            if now - self.last_log_t >= self.log_interval:
                self.get_logger().info(f"Red coefficient: {red_ratio:.6f}")
                self.last_log_t = now

            # Overlay the red coefficient
            h, w = frame.shape[:2]
            text = f"Red coef: {red_ratio:.6f}"
            cv2.putText(frame, text, (10, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)

            # Display the processed frame
            try:
                cv2.imshow("Red Coefficient Detector", frame)
                cv2.imshow("Red Mask", red_mask)
                cv2.waitKey(1)
            except Exception as e:
                # Ignore display errors - this may run headless
                pass
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    detector = ElevatorRedCoefficient()
    rclpy.spin(detector)
    
    # Cleanup
    cv2.destroyAllWindows()
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 