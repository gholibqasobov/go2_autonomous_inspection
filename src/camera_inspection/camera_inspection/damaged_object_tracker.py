import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
from deep_sort_realtime.deepsort_tracker import DeepSort
import json
import os
from datetime import datetime
import math

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
        # Subscribe to depth image
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )
        # Subscribe to servo angles
        self.servo_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.servo_callback,
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
        self.PROXIMITY_THRESH = 0.3         # Distance threshold in meters to consider objects as duplicates
        
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
        
        # New state for depth and servo
        self.depth_image = None
        self.servo_angles = {}  # Store pan, tilt angles of camera servo
        
        # Store positions of saved objects by class for proximity check
        self.saved_object_positions = {}  # {class_name: [(x, y, z), ...]}
        
        # Camera intrinsic parameters for RealSense D435i
        # These are approximate - you may need to update with your specific camera calibration
        self.fx = 615.0  # focal length x
        self.fy = 615.0  # focal length y
        self.cx = 320.0  # optical center x
        self.cy = 240.0  # optical center y
        
        # Create a directory to store object images and data
        self.output_dir = "src/camera_inspection/damaged_objects"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Create or load JSON file for storing object data
        self.json_file = os.path.join(self.output_dir, "damaged_objects_data.json")
        if os.path.exists(self.json_file):
            with open(self.json_file, 'r') as f:
                self.objects_data = json.load(f)
                # Initialize saved positions from loaded data
                self._init_saved_positions_from_data()
        else:
            self.objects_data = []
    
    def _init_saved_positions_from_data(self):
        """Initialize the saved positions dictionary from loaded object data"""
        for obj in self.objects_data:
            class_name = obj.get('class_name')
            if class_name and 'position' in obj:
                pos = obj['position']
                position = (pos['x'], pos['y'], pos['z'])
                
                if class_name not in self.saved_object_positions:
                    self.saved_object_positions[class_name] = []
                    
                self.saved_object_positions[class_name].append(position)
                
                # Also mark the ID as saved
                obj_id = obj.get('id')
                if obj_id is not None:
                    self.saved_ids.add(obj_id)
    
    def is_close_to_existing_object(self, class_name, position):
        """
        Check if a detected object is too close to an existing saved object of the same class.
        
        Args:
            class_name: The class name of the object
            position: Tuple of (x, y, z) position
            
        Returns:
            bool: True if too close to an existing object, False otherwise
        """
        if position is None:
            return False
            
        if class_name not in self.saved_object_positions:
            return False
            
        # Check distance to all saved objects of the same class
        for saved_pos in self.saved_object_positions[class_name]:
            distance = np.sqrt(
                (position[0] - saved_pos[0])**2 + 
                (position[1] - saved_pos[1])**2 + 
                (position[2] - saved_pos[2])**2
            )
            
            if distance < self.PROXIMITY_THRESH:
                self.get_logger().info(
                    f"Object of class {class_name} at {position} is too close "
                    f"(distance={distance:.2f}m) to existing object at {saved_pos}"
                )
                return True
                
        return False
        
    def depth_callback(self, msg):
        """Store the most recent depth image"""
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {str(e)}")
    
    def servo_callback(self, msg):
        """Store the servo angles from joint states"""
        try:
            # Store servo angles based on joint names
            # Update these joint names to match your actual servo joint names
            for i, name in enumerate(msg.name):
                if 'pan' in name.lower():
                    self.servo_angles['pan'] = msg.position[i]
                elif 'tilt' in name.lower():
                    self.servo_angles['tilt'] = msg.position[i]
            
            self.get_logger().debug(f"Servo angles: {self.servo_angles}")
        except Exception as e:
            self.get_logger().error(f"Error processing servo angles: {str(e)}")
    
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
    
    def get_3d_position(self, bbox, depth_img):
        """
        Calculate 3D position of an object given its bounding box and depth image
        
        Args:
            bbox: Tuple of (x1, y1, x2, y2) bounding box coordinates
            depth_img: Depth image aligned with color image
            
        Returns:
            Tuple of (x, y, z) object position in camera frame
        """
        if depth_img is None:
            self.get_logger().warn("No depth image available")
            return None
        
        x1, y1, x2, y2 = bbox
        
        # Get center of bounding box
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        
        # Ensure coordinates are within bounds
        height, width = depth_img.shape
        if not (0 <= center_x < width and 0 <= center_y < height):
            self.get_logger().warn(f"Invalid bbox center: ({center_x}, {center_y})")
            return None
        
        # Get depth at center point - average a small region to be more stable
        region_size = 5
        x_min = max(0, center_x - region_size)
        x_max = min(width, center_x + region_size)
        y_min = max(0, center_y - region_size)
        y_max = min(height, center_y + region_size)
        
        # Extract depth region and filter out zero/invalid values
        depth_region = depth_img[y_min:y_max, x_min:x_max]
        valid_depths = depth_region[depth_region > 0]
        
        if len(valid_depths) == 0:
            self.get_logger().warn("No valid depth values in region")
            return None
        
        # Get median depth value in millimeters, convert to meters
        depth_mm = np.median(valid_depths)
        depth_m = depth_mm / 1000.0
        
        # Calculate 3D coordinates in camera frame
        # Convert from pixel coordinates to camera coordinates
        x = (center_x - self.cx) * depth_m / self.fx
        y = (center_y - self.cy) * depth_m / self.fy
        z = depth_m
        
        # Transform from camera to world frame using servo angles
        # This is a simplified transformation - you may need a more complex one
        # depending on your robot's configuration
        if 'pan' in self.servo_angles and 'tilt' in self.servo_angles:
            pan = self.servo_angles['pan']
            tilt = self.servo_angles['tilt']
            
            # Apply tilt (rotation around X axis)
            y_tilt = y * math.cos(tilt) - z * math.sin(tilt)
            z_tilt = y * math.sin(tilt) + z * math.cos(tilt)
            
            # Apply pan (rotation around Y axis)
            x_pan = x * math.cos(pan) + z_tilt * math.sin(pan)
            z_pan = -x * math.sin(pan) + z_tilt * math.cos(pan)
            
            # Final position in world frame
            world_x = x_pan + self.robot_position[0]
            world_y = y_tilt + self.robot_position[1]
            world_z = z_pan + self.robot_position[2]
            
            return (world_x, world_y, world_z)
        else:
            self.get_logger().warn("No servo angles available for transformation")
            # Fall back to camera frame if no servo angles
            return (x, y, z)
    
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

                # Calculate 3D position of object
                object_position = self.get_3d_position((x1, y1, x2, y2), self.depth_image)
                
                # Check proximity to existing objects of the same class
                is_duplicate = False
                if object_position:
                    is_duplicate = self.is_close_to_existing_object(class_name, object_position)

                # Check if this is a new object that we haven't saved yet
                is_new_object = tid not in self.saved_ids and not is_duplicate

                # Add to detected objects with class info
                object_data = {
                    'id': tid,
                    'class_id': cls_id,
                    'class_name': class_name,
                    'bbox': (x1, y1, x2, y2),
                    'object_position': object_position,
                    'robot_position': self.robot_position,
                    'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    'is_new': is_new_object,
                    'is_duplicate': is_duplicate
                }
                detected_objects.append(object_data)
                
                # Draw on frame for visualization
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Add text with different color based on whether it's a new object
                if is_duplicate:
                    color = (0, 165, 255)  # Orange for duplicates
                    status = "DUPLICATE"
                elif is_new_object:
                    color = (0, 0, 255)  # Red for new
                    status = "NEW"
                else:
                    color = (0, 255, 0)  # Green for known
                    status = "KNOWN"
                    
                text = f"{class_name} ID:{tid} ({status})"
                
                # Add position info if available
                if object_position:
                    x, y, z = object_position
                    text += f" | Pos: ({x:.2f}, {y:.2f}, {z:.2f})"
                    
                cv2.putText(frame, text, (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # Log tracking information and save new objects
            if detected_objects:
                new_objects_count = sum(1 for obj in detected_objects if obj['is_new'])
                duplicate_count = sum(1 for obj in detected_objects if obj['is_duplicate'])
                self.get_logger().info(
                    f"Tracking {len(detected_objects)} objects, {new_objects_count} new, "
                    f"{duplicate_count} duplicates, {len(self.unique_ids)} unique IDs total"
                )
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
            class_name = obj.get('class_name')
            object_position = obj.get('object_position')
            
            # Only save objects we haven't saved before and that aren't duplicates
            if obj_id in self.saved_ids or obj.get('is_duplicate', False):
                continue
                
            # Mark as saved
            self.saved_ids.add(obj_id)
            
            # Add to saved positions for future proximity checks
            if object_position and class_name:
                if class_name not in self.saved_object_positions:
                    self.saved_object_positions[class_name] = []
                self.saved_object_positions[class_name].append(object_position)
            
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
                'class_name': class_name,
                'image_file': img_filename,
                'timestamp': obj['timestamp'],
                'bbox': {
                    'x1': x1,
                    'y1': y1,
                    'x2': x2,
                    'y2': y2
                }
            }
            
            # Add 3D position if available
            if object_position:
                x, y, z = object_position
                object_data['position'] = {
                    'x': x,
                    'y': y,
                    'z': z
                }
            else:
                # Fall back to robot position if object position not available
                object_data['robot_position'] = {
                    'x': obj['robot_position'][0],
                    'y': obj['robot_position'][1],
                    'z': obj['robot_position'][2]
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