import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
from deep_sort_realtime.deepsort_tracker import DeepSort
import json
import os
from datetime import datetime

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

        # Create a publisher for annotated images
        self.annotated_image_pub = self.create_publisher(Image, '/annotated_image', 10) # To visualize in Rviz
        
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
        self.saved_ids = set()  # Track which objects we've already saved
        self.last_gray = None
        self.robot_position = (0, 0, 0)  # x, y, z
        
        # Create a directory to store object images and data
        self.output_dir = "src/camera_inspection/damaged_objects"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Create or load JSON file for storing object data
        self.json_file = os.path.join(self.output_dir, "damaged_objects_data.json")
        if os.path.exists(self.json_file):
            with open(self.json_file, 'r') as f:
                self.objects_data = json.load(f)
        else:
            self.objects_data = []
    
    def save_robot_odom(self, msg):
        """Callback function that gets called when a new TF message is received."""
        try:
            x, y, z = msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y, msg.transforms[0].transform.translation.z
            self.robot_position = (x, y, z)
            self.get_logger().info(f"x is {x}, y is {y}, z is {z}")
        except IndexError:
            self.get_logger().warn("No transforms in TF message")
        except Exception as e:
            self.get_logger().error(f"Error processing TF message: {str(e)}")
    
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

                # Get class information
                cls_id = t.det_class
                class_name = self.model.names.get(cls_id, str(cls_id))

                # Check if this is a new object that we haven't saved yet
                is_new_object = tid not in self.saved_ids

                # Add to detected objects with class info
                object_data = {
                    'id': tid,
                    'class_id': cls_id,
                    'class_name': class_name,
                    'bbox': (x1, y1, x2, y2),
                    'robot_position': self.robot_position,
                    'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    'is_new': is_new_object
                }
                detected_objects.append(object_data)
                
                # Draw on frame for visualization
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Add text with different color based on whether it's a new object
                color = (0, 0, 255) if is_new_object else (0, 255, 0)
                status = "NEW" if is_new_object else "KNOWN"
                cv2.putText(frame, f"{class_name} ID:{tid} ({status})", (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # Log tracking information and save new objects
            if detected_objects:
                self.get_logger().info(f"Tracking {len(detected_objects)} objects, {len(self.unique_ids)} unique IDs total")
                self.save_damaged_object_poses(detected_objects, frame)
            
            # Add more information to the annotated image
            cv2.putText(frame, f"Unique objects: {len(self.unique_ids)}", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
            cv2.putText(frame, f"Saved objects: {len(self.saved_ids)}", (20, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            
            # Publish the annotated image
            annotated_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
    
    def save_damaged_object_poses(self, detected_objects, frame):
        """Save new detected objects' images and poses to a JSON file."""
        new_objects = [obj for obj in detected_objects if obj['is_new']]
        
        if not new_objects:
            return
        
        self.get_logger().info(f"Saving {len(new_objects)} new damaged objects")
        
        # Get current timestamp for filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        for obj in new_objects:
            obj_id = obj['id']
            x1, y1, x2, y2 = obj['bbox']
            
            # Only save objects we haven't saved before
            if obj_id in self.saved_ids:
                continue
                
            # Mark as saved
            self.saved_ids.add(obj_id)
            
            # Extract object image
            # obj_img = frame[y1:y2, x1:x2]
            obj_img = frame
            if obj_img.size == 0:
                self.get_logger().warn(f"Empty region for object ID {obj_id}, skipping")
                continue
                
            # Create filename for the object image
            img_filename = f"object_{obj_id}_{timestamp}.jpg"
            img_path = os.path.join(self.output_dir, img_filename)
            
            # Save the image
            cv2.imwrite(img_path, obj_img)
            
            # Prepare data for JSON
            object_data = {
                'id': obj_id,
                'class_name': obj.get('class_name'),
                'image_file': img_filename,
                'robot_position': {
                    'x': obj['robot_position'][0],
                    'y': obj['robot_position'][1],
                    'z': obj['robot_position'][2]
                },
                'timestamp': obj['timestamp'],
                'bbox': {
                    'x1': x1,
                    'y1': y1,
                    'x2': x2,
                    'y2': y2
                }
            }
            
            # Add to objects data
            self.objects_data.append(object_data)
            
            self.get_logger().info(f"Saved object ID {obj_id} to {img_path}")
        
        # Save updated JSON file
        with open(self.json_file, 'w') as f:
            json.dump(self.objects_data, f, indent=2)

def main(args=None):
    rclpy.init(args=args)
    damaged_object_tracker = DamagedObjectTracker()
    rclpy.spin(damaged_object_tracker)
    damaged_object_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()