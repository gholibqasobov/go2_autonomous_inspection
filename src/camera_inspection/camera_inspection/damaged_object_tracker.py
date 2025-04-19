import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
from collections import defaultdict
import numpy as np
from deep_sort_realtime.deepsort_tracker import DeepSort

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
        self.model = YOLO('src/camera_inspection/model/damaged_object_yolo.pt')
        
        # Object tracking parameters
        self.CONF_THRESH = 0.5              # min detection confidence
        self.STATIONARY_THRESH = 5000       # max # of changed pixels to be 'stationary'
        self.DIFF_THRESH = 25               # per-pixel diff threshold
        
        # Initialize tracker
        self.tracker = DeepSort(
            max_age=30,       # frames to keep 'dead' tracks
            n_init=3,         # frames until track is confirmed
            max_cosine_distance=0.2
        )
        
        # Tracking state
        self.unique_ids = set()
        self.last_gray = None
        self.robot_position = (0, 0, 0)  # x, y, z
        
        # Create a publisher for annotated images
        self.annotated_image_pub = self.create_publisher(Image, '/annotated_image', 10)
    
    def save_robot_odom(self, msg):
        """Callback function that gets called when a new TF message is received."""
        # self.get_logger().info(f'Received TF message with {msg.transforms}')
        x, y, z = msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y, msg.transforms[0].transform.translation.z
        self.robot_position = (x, y, z)
        self.get_logger().info(f"x is {x}, y is {y}, z is {z}")
        return x, y, z
    
    def save_damaged_image(self, msg):
        """Process camera images and track damaged objects."""
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 1) Check if we're stationary
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if self.last_gray is None:
                self.last_gray = gray
                return
                
            diff = cv2.absdiff(gray, self.last_gray)
            moving_pixels = np.count_nonzero(diff > self.DIFF_THRESH)
            self.last_gray = gray
            
            if moving_pixels >= self.STATIONARY_THRESH:
                self.get_logger().info("Moving - waiting for stability")
                return
            
            # 2) Run YOLO → get raw xyxy boxes + scores + classes
            results = self.model(frame)[0]
            xyxy = results.boxes.xyxy.cpu().numpy()    # (N,4): x1,y1,x2,y2
            scores = results.boxes.conf.cpu().numpy()  # (N,)
            classes = results.boxes.cls.cpu().numpy()  # (N,)
            
            # 3) Convert to DeepSORT's ([x,y,w,h], score, cls) tuples
            raw_dets = []
            for (x1, y1, x2, y2), conf, cls in zip(xyxy, scores, classes):
                if conf < self.CONF_THRESH:
                    continue
                w = x2 - x1
                h = y2 - y1
                bbox_xywh = [float(x1), float(y1), float(w), float(h)]
                raw_dets.append((bbox_xywh, float(conf), int(cls)))
            
            # 4) Update DeepSORT
            tracks = self.tracker.update_tracks(raw_dets, frame=frame)
            
            # 5) Process tracks
            detected_objects = []
            for t in tracks:
                if not t.is_confirmed():
                    continue
                    
                tid = t.track_id
                self.unique_ids.add(tid)
                
                # Extract bounding box
                x1, y1, x2, y2 = map(int, t.to_ltrb())
                
                # Add to detected objects
                detected_objects.append({
                    'id': tid,
                    'bbox': (x1, y1, x2, y2),
                    'robot_position': self.robot_position,
                    'frame_timestamp': msg.header.stamp
                })
                
                # Draw on frame for visualization
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"ID:{tid}", (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Log tracking information
            if detected_objects:
                self.get_logger().info(f"Tracking {len(detected_objects)} objects, {len(self.unique_ids)} unique IDs total")
                
                # Here you could add code to save object positions in world coordinates
                self.save_damaged_object_poses(detected_objects)
            
            # Add more information to the annotated image
            cv2.putText(frame, f"Unique objects: {len(self.unique_ids)}", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
            
            # Local visualization can be enabled for debugging
            # cv2.imshow("Damaged Object Tracking", frame)
            # cv2.waitKey(1)
            
            # Publish the annotated image
            annotated_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
    
    def save_damaged_object_poses(self, detected_objects):
        """Save the poses of detected damaged objects."""
        # This is where you would implement logic to:
        # 1. Convert pixel coordinates to 3D coordinates
        # 2. Transform from camera frame to world frame
        # 3. Store or publish the detected object poses
        
        # Example implementation (placeholder)
        for obj in detected_objects:
            obj_id = obj['id']
            bbox = obj['bbox']
            robot_pos = obj['robot_position']
            
            # Calculate centroid of the bounding box
            cx = (bbox[0] + bbox[2]) / 2
            cy = (bbox[1] + bbox[3]) / 2
            
            # For a full implementation, you would:
            # - Use depth information if available
            # - Apply camera intrinsics
            # - Transform to world coordinates
            
            self.get_logger().info(f"Object ID {obj_id} detected at pixel ({cx}, {cy})")
            self.get_logger().info(f"Robot was at position {robot_pos}")
            
            # Add your code to store this information in a database or publish it

def main(args=None):
    rclpy.init(args=args)
    damaged_object_tracker = DamagedObjectTracker()
    rclpy.spin(damaged_object_tracker)
    damaged_object_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()