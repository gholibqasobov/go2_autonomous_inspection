import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
from deep_sort_realtime.deepsort_tracker import DeepSort
import json
import os
from datetime import datetime
from collections import deque

class DepthMeasurementTracker(Node):
    def __init__(self):
        super().__init__('depth_measurement_tracker')
        
        # Subscribe to camera color image
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.process_image,
            10
        )
        
        # Subscribe to depth image
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        # Create a publisher for annotated images
        self.annotated_image_pub = self.create_publisher(Image, '/annotated_image', 10)
        
        self.cv_bridge = CvBridge()
        self.model = YOLO('src/camera_inspection/model/damaged_object_yolo.pt')
        
        # Object tracking parameters
        self.CONF_THRESH = 0.5
        self.PROXIMITY_THRESHOLD = 0.3  # Distance in meters to consider objects as duplicates
        
        # Initialize tracker
        self.tracker = DeepSort(
            max_age=30,
            n_init=3,
            max_cosine_distance=0.2
        )
        
        # Tracking state
        self.unique_ids = set()
        
        # Position filtering parameters
        self.MIN_CONFIDENCE_MEASUREMENTS = 5    # Minimum measurements to finalize a position
        self.MAX_POSITION_HISTORY = 10          # Maximum measurements to keep in history
        self.POSITION_STABILITY_THRESHOLD = 0.05 # Maximum allowed variance in meters
        
        # Store last known positions of tracked objects
        self.object_positions = {}              # {track_id: (x, y, z)} - current positions
        self.position_history = {}              # {track_id: deque((x,y,z), confidence, timestamp)}
        self.finalized_positions = {}           # {track_id: (x, y, z)} - definitive positions
        self.saved_object_ids = set()           # Track IDs of objects we've already saved
        
        # Store saved object positions by class for proximity check
        self.saved_positions_by_class = {}      # {class_name: [(x, y, z), ...]}
        
        # Depth image storage
        self.depth_image = None
        self.color_image_shape = None
        self.depth_image_shape = None
        
        # Camera intrinsic parameters for RealSense D435i
        # Update these with your camera's actual calibration parameters
        self.fx = 615.0  # focal length x
        self.fy = 615.0  # focal length y
        self.cx = 320.0  # optical center x
        self.cy = 240.0  # optical center y
        
        # Create a directory to store object images and data
        self.output_dir = "src/camera_inspection/depth_measurements"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Create or load JSON file for storing object data
        self.json_file = os.path.join(self.output_dir, "depth_measurements.json")
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
            if class_name and 'camera_frame_position' in obj:
                pos = obj['camera_frame_position']
                position = (pos['x'], pos['y'], pos['z'])
                
                if class_name not in self.saved_positions_by_class:
                    self.saved_positions_by_class[class_name] = []
                    
                self.saved_positions_by_class[class_name].append(position)
                
                # Also mark the ID as saved
                obj_id = obj.get('id')
                if obj_id is not None:
                    self.saved_object_ids.add(obj_id)
    
    def is_too_close_to_existing_object(self, class_name, position):
        """
        Check if position is too close to an existing saved object of the same class
        
        Args:
            class_name: Class name of the object
            position: Tuple of (x, y, z) position
            
        Returns:
            bool: True if too close to existing object of same class, False otherwise
        """
        if class_name not in self.saved_positions_by_class:
            return False
            
        for saved_pos in self.saved_positions_by_class[class_name]:
            # Calculate 3D Euclidean distance
            dx = position[0] - saved_pos[0]
            dy = position[1] - saved_pos[1]
            dz = position[2] - saved_pos[2]
            distance = np.sqrt(dx*dx + dy*dy + dz*dz)
            
            if distance < self.PROXIMITY_THRESHOLD:
                self.get_logger().info(
                    f"Object of class {class_name} is too close (distance={distance:.2f}m) "
                    f"to existing object at {saved_pos}"
                )
                return True
                
        return False
    
    def depth_callback(self, msg):
        """Store the most recent depth image"""
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_image_shape = self.depth_image.shape
            self.get_logger().debug(f"Depth image shape: {self.depth_image_shape}")
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {str(e)}")
    
    def scale_coordinates(self, x, y, from_shape, to_shape):
        """
        Scale coordinates from one image resolution to another
        
        Args:
            x, y: The coordinates to scale
            from_shape: The source resolution (height, width)
            to_shape: The target resolution (height, width)
            
        Returns:
            Scaled coordinates as integers
        """
        if from_shape is None or to_shape is None:
            return x, y
            
        from_height, from_width = from_shape
        to_height, to_width = to_shape
        
        # Scale coordinates
        scaled_x = int(x * (to_width / from_width))
        scaled_y = int(y * (to_height / from_height))
        
        # Ensure coordinates are within bounds
        scaled_x = max(0, min(scaled_x, to_width - 1))
        scaled_y = max(0, min(scaled_y, to_height - 1))
        
        return scaled_x, scaled_y
    
    def get_object_distance(self, bbox, depth_img, color_shape, detection_conf=1.0):
        """
        Calculate distance to an object given its bounding box and depth image
        
        Args:
            bbox: Tuple of (x1, y1, x2, y2) bounding box coordinates in color image space
            depth_img: Depth image aligned with color image
            color_shape: Shape of the color image (height, width)
            detection_conf: Detection confidence score (0.0-1.0)
            
        Returns:
            Tuple of (x, y, z) object position in camera frame and measurement confidence
        """
        if depth_img is None:
            self.get_logger().warn("No depth image available")
            return None, 0.0
        
        x1, y1, x2, y2 = bbox
        depth_shape = depth_img.shape
        
        # Get center of bounding box in color image coordinates
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        
        # Calculate box size - larger objects provide more reliable depth
        box_width = x2 - x1
        box_height = y2 - y1
        box_size = min(box_width, box_height)
        size_confidence = min(1.0, box_size / 200.0)  # Normalize, cap at 1.0
        
        self.get_logger().debug(f"Original center: ({center_x}, {center_y})")
        
        # Scale center coordinates from color to depth image
        depth_center_x, depth_center_y = self.scale_coordinates(
            center_x, center_y, color_shape, depth_shape
        )
        
        self.get_logger().debug(f"Scaled center: ({depth_center_x}, {depth_center_y})")
        
        # Double-check bounds after scaling
        height, width = depth_shape
        if not (0 <= depth_center_x < width and 0 <= depth_center_y < height):
            self.get_logger().warn(
                f"Invalid bbox center after scaling: color ({center_x}, {center_y}) → "
                f"depth ({depth_center_x}, {depth_center_y}), depth shape: {depth_shape}"
            )
            return None, 0.0
        
        # Get depth at center point - average a small region to be more stable
        region_size = 5
        x_min = max(0, depth_center_x - region_size)
        x_max = min(width, depth_center_x + region_size)
        y_min = max(0, depth_center_y - region_size)
        y_max = min(height, depth_center_y + region_size)
        
        # Extract depth region and filter out zero/invalid values
        depth_region = depth_img[y_min:y_max, x_min:x_max]
        valid_depths = depth_region[depth_region > 0]
        
        if len(valid_depths) == 0:
            self.get_logger().warn("No valid depth values in region")
            return None, 0.0
        
        # Calculate depth reliability based on number of valid pixels
        region_size_total = (x_max - x_min) * (y_max - y_min)
        depth_reliability = len(valid_depths) / region_size_total
        
        # Get median depth value in millimeters, convert to meters
        depth_mm = np.median(valid_depths)
        depth_m = depth_mm / 1000.0
        
        # Check if depth is within reasonable range (0.1m to 10m)
        if not (0.1 <= depth_m <= 10.0):
            self.get_logger().warn(f"Unreasonable depth value: {depth_m}m")
            depth_reliability *= 0.5  # Reduce confidence
        
        # Calculate 3D coordinates in camera frame
        # Use the properly scaled coordinates for accurate projection
        x = (depth_center_x - self.cx) * depth_m / self.fx
        y = (depth_center_y - self.cy) * depth_m / self.fy
        z = depth_m
        
        # Calculate overall confidence in this measurement
        # Combine detection confidence, depth reliability, and object size
        confidence = detection_conf * depth_reliability * size_confidence
        
        return (x, y, z), confidence
    
    def update_position_history(self, track_id, position, confidence, timestamp):
        """
        Update position history for a tracked object and determine if we have a finalized position
        
        Args:
            track_id: Unique ID of the tracked object
            position: Tuple of (x, y, z) position
            confidence: Confidence score of this measurement (0.0-1.0)
            timestamp: Timestamp of measurement
            
        Returns:
            True if position is finalized, False otherwise
        """
        # Skip if we already have a finalized position
        if track_id in self.finalized_positions:
            return True
            
        # Initialize history if needed
        if track_id not in self.position_history:
            self.position_history[track_id] = deque(maxlen=self.MAX_POSITION_HISTORY)
        
        # Add current position to history
        self.position_history[track_id].append((position, confidence, timestamp))
        
        # Only finalize if we have enough measurements
        if len(self.position_history[track_id]) < self.MIN_CONFIDENCE_MEASUREMENTS:
            return False
            
        # Check if position has stabilized
        # Get weighted average of all positions in history
        positions = [p for p, _, _ in self.position_history[track_id]]
        confidences = [c for _, c, _ in self.position_history[track_id]]
        
        # Calculate position variances
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        zs = [p[2] for p in positions]
        
        # Check if position is stable enough based on variance
        x_var = np.var(xs)
        y_var = np.var(ys)
        z_var = np.var(zs)
        
        position_stable = (x_var < self.POSITION_STABILITY_THRESHOLD and
                           y_var < self.POSITION_STABILITY_THRESHOLD and
                           z_var < self.POSITION_STABILITY_THRESHOLD)
        
        if position_stable:
            # Calculate weighted average as final position
            total_weight = sum(confidences)
            if total_weight > 0:
                # Weighted sum
                x_weighted = sum(p[0] * c for p, c in zip(positions, confidences)) / total_weight
                y_weighted = sum(p[1] * c for p, c in zip(positions, confidences)) / total_weight
                z_weighted = sum(p[2] * c for p, c in zip(positions, confidences)) / total_weight
                
                # Save finalized position
                self.finalized_positions[track_id] = (x_weighted, y_weighted, z_weighted)
                self.get_logger().info(
                    f"Finalized position for object {track_id}: "
                    f"({x_weighted:.3f}, {y_weighted:.3f}, {z_weighted:.3f})"
                )
                return True
        
        return False
    
    def process_image(self, msg):
        """Process camera images, track objects and measure distances."""
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.color_image_shape = frame.shape[:2]  # Store color image shape (height, width)
            
            # Run YOLO → get raw xyxy boxes + scores + classes
            results = self.model(frame)[0]
            xyxy = results.boxes.xyxy.cpu().numpy()
            scores = results.boxes.conf.cpu().numpy()
            classes = results.boxes.cls.cpu().numpy()
            
            # Track which objects were actually detected in this frame (not just tracked)
            detected_ids = set()
            
            # Store detection bounding boxes to check if tracks have current detections
            detection_boxes = []
            for (x1, y1, x2, y2), conf, cls in zip(xyxy, scores, classes):
                if conf < self.CONF_THRESH:
                    continue
                detection_boxes.append((x1, y1, x2, y2))
                
            # Convert to DeepSORT's ([x,y,w,h], score, cls) tuples
            raw_dets = []
            for (x1, y1, x2, y2), conf, cls in zip(xyxy, scores, classes):
                if conf < self.CONF_THRESH:
                    continue
                w = x2 - x1
                h = y2 - y1
                bbox_xywh = [float(x1), float(y1), float(w), float(h)]
                raw_dets.append((bbox_xywh, float(conf), int(cls)))
            
            # Update DeepSORT
            tracks = self.tracker.update_tracks(raw_dets, frame=frame)
            
            # Process tracks
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
                
                # Get current timestamp
                timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                
                # Check if this track has a detection in this frame
                # Use time_since_update == 0 to determine if there's a detection in the current frame
                has_detection = t.time_since_update == 0
                
                # Check if we already have a finalized position
                has_finalized_position = tid in self.finalized_positions
                
                # Get position information
                is_too_close = False
                if has_finalized_position:
                    # Use finalized position
                    object_position = self.finalized_positions[tid]
                    position_status = "FINALIZED"
                    is_finalized = True
                    
                    # Check if this object is too close to an existing saved object of the same class
                    is_too_close = self.is_too_close_to_existing_object(class_name, object_position)
                elif has_detection:
                    detected_ids.add(tid)
                    # Calculate distance to object only for newly detected objects
                    object_position, confidence = self.get_object_distance(
                        (x1, y1, x2, y2), 
                        self.depth_image, 
                        self.color_image_shape,
                        detection_conf=float(t.det_conf) if hasattr(t, 'det_conf') else 1.0
                    )
                    
                    if object_position:
                        # Update the current position
                        self.object_positions[tid] = object_position
                        
                        # Update position history and check if it's now finalized
                        is_finalized = self.update_position_history(tid, object_position, confidence, timestamp)
                        
                        if is_finalized and tid not in self.finalized_positions:
                            # Use the finalized position from now on
                            object_position = self.finalized_positions[tid]
                            position_status = "FINALIZED"
                            
                            # Check if this object is too close to an existing saved object of the same class
                            is_too_close = self.is_too_close_to_existing_object(class_name, object_position)
                        else:
                            position_status = "UPDATING"
                    else:
                        # No valid position from depth measurement
                        object_position = self.object_positions.get(tid)
                        position_status = "TRACKING"
                        is_finalized = False
                else:
                    # Use the last known position if available
                    object_position = self.object_positions.get(tid)
                    position_status = "TRACKING"
                    is_finalized = False
                
                # Add to detected objects with class info
                object_data = {
                    'id': tid,
                    'class_name': class_name,
                    'bbox': (x1, y1, x2, y2),
                    'camera_frame_position': object_position,
                    'has_detection': has_detection,
                    'is_finalized': has_finalized_position or is_finalized,
                    'position_status': position_status,
                    'is_too_close': is_too_close,
                    'timestamp': timestamp
                }
                detected_objects.append(object_data)
                
                # Draw on frame for visualization
                # Use different colors based on position status and proximity
                if is_too_close:
                    box_color = (0, 165, 255)  # Orange for too close objects
                    text_color = (0, 165, 255)
                    label = "TOO CLOSE"
                elif position_status == "FINALIZED":
                    box_color = (255, 0, 255)  # Magenta for finalized positions
                    text_color = (255, 0, 255)
                    label = position_status
                elif has_detection:
                    box_color = (0, 255, 0)    # Green for detected objects
                    text_color = (0, 0, 255)
                    label = position_status
                else:
                    box_color = (255, 165, 0)  # Orange for tracked-only objects
                    text_color = (255, 165, 0)
                    label = position_status
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
                
                text = f"{class_name} ID:{tid} ({label})"
                
                # Add position info if available
                if object_position:
                    x, y, z = object_position
                    distance_text = f"Dist: {z:.2f}m | 3D: ({x:.2f}, {y:.2f}, {z:.2f})"
                    if has_detection:
                        self.get_logger().info(f"Object {tid} ({class_name}): {distance_text} - {label}")
                    text += f" | {distance_text}"
                
                cv2.putText(frame, text, (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
            
            # Clean up positions for tracks that are no longer active
            active_track_ids = {t.track_id for t in tracks if t.is_confirmed()}
            for tid in list(self.object_positions.keys()):
                if tid not in active_track_ids:
                    # Don't remove finalized positions
                    if tid not in self.finalized_positions:
                        del self.object_positions[tid]
                    # Remove position history for disappeared tracks
                    if tid in self.position_history:
                        del self.position_history[tid]
            
            # Add more information to the annotated image
            cv2.putText(frame, f"Unique objects: {len(self.unique_ids)}", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
            cv2.putText(frame, f"Detected in frame: {len(detected_ids)}", (20, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            cv2.putText(frame, f"Finalized positions: {len(self.finalized_positions)}", (20, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 255), 2)
            
            # Add resolution information to help debug
            if self.color_image_shape and self.depth_image_shape:
                resolution_text = f"Color: {self.color_image_shape[1]}x{self.color_image_shape[0]}, Depth: {self.depth_image_shape[1]}x{self.depth_image_shape[0]}"
                cv2.putText(frame, resolution_text, (20, 150),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Publish the annotated image
            annotated_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)
            
            # Save data only for objects with finalized positions that haven't been saved yet
            # and are not too close to existing objects of the same class
            objects_to_save = [obj for obj in detected_objects 
                              if obj['is_finalized'] and 
                                 obj['id'] not in self.saved_object_ids and
                                 not obj['is_too_close']]
            
            if objects_to_save:
                self.save_measurements(objects_to_save, frame)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
    
    def save_measurements(self, detected_objects, frame):
        """Save detected objects' images and distance measurements to a JSON file."""
        if not detected_objects:
            return
        
        # Get current timestamp for filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        for obj in detected_objects:
            obj_id = obj['id']
            x1, y1, x2, y2 = obj['bbox']
            class_name = obj.get('class_name')
            object_position = obj.get('camera_frame_position')
            
            # Skip objects without valid distance measurements
            if object_position is None:
                continue
                
            # Mark as saved
            self.saved_object_ids.add(obj_id)
            
            # Add to saved positions for future proximity checks
            if class_name not in self.saved_positions_by_class:
                self.saved_positions_by_class[class_name] = []
            self.saved_positions_by_class[class_name].append(object_position)
                
            # Extract object image
            obj_img = frame
            
            # Create filename for the object image
            img_filename = f"depth_measurement_{obj_id}_{timestamp}.jpg"
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
            
            # Add camera frame position
            x, y, z = object_position
            object_data['camera_frame_position'] = {
                'x': x,
                'y': y,
                'z': z  # z is the direct distance to the camera
            }
            
            # Add to objects data
            self.objects_data.append(object_data)
            
            self.get_logger().info(f"Saved finalized measurement for object ID {obj_id} to {img_path}")
        
        # Save updated JSON file
        with open(self.json_file, 'w') as f:
            json.dump(self.objects_data, f, indent=2)

def main(args=None):
    rclpy.init(args=args)
    depth_measurement_tracker = DepthMeasurementTracker()
    rclpy.spin(depth_measurement_tracker)
    depth_measurement_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 