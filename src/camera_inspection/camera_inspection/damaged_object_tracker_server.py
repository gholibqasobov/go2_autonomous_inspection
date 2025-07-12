import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
from deep_sort_realtime.deepsort_tracker import DeepSort
import json
import os
from datetime import datetime
import math
from std_srvs.srv import SetBool, Trigger
from threading import Lock

class DamagedObjectTrackerServer(Node):
    def __init__(self):
        super().__init__('damaged_object_tracker_server')
        
        # Use reentrant callback group for concurrent processing
        self.callback_group = ReentrantCallbackGroup()
        
        # Service servers
        self.start_stop_service = self.create_service(
            SetBool,
            '~/start_inspection',
            self.start_stop_callback,
            callback_group=self.callback_group
        )
        
        self.status_service = self.create_service(
            Trigger,
            '~/get_status',
            self.status_callback,
            callback_group=self.callback_group
        )
        
        # Inspection state
        self.inspection_active = False
        self.inspection_lock = Lock()
        
        # Subscriptions (initially disabled)
        self.robot_odom_sub = None
        self.camera_sub = None
        self.depth_sub = None
        self.servo_sub = None
        
        # Create a publisher for annotated images
        self.annotated_image_pub = self.create_publisher(Image, '/annotated_image', 10)
        
        # Initialize CV bridge and YOLO model
        self.cv_bridge = CvBridge()
        self.model = YOLO('src/camera_inspection/model/damaged_object_yolo.pt')
        
        # Object tracking parameters
        self.CONF_THRESH = 0.5
        self.STATIONARY_THRESH = 5000
        self.DIFF_THRESH = 25
        self.PROXIMITY_THRESH = 0.3
        
        # Initialize tracker
        self.tracker = DeepSort(
            max_age=30,
            n_init=3,
            max_cosine_distance=0.2
        )
        
        # Tracking state
        self.unique_ids = set()
        self.saved_ids = set()
        self.last_gray = None
        self.robot_position = (0, 0, 0)
        
        # Depth and servo state
        self.depth_image = None
        self.servo_angles = {}
        
        # Store positions of saved objects by class for proximity check
        self.saved_object_positions = {}
        
        # Camera intrinsic parameters for RealSense D435i
        self.fx = 615.0
        self.fy = 615.0
        self.cx = 320.0
        self.cy = 240.0
        
        # Create output directory
        self.output_dir = "src/camera_inspection/damaged_objects"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Load existing data
        self.json_file = os.path.join(self.output_dir, "damaged_objects_data.json")
        if os.path.exists(self.json_file):
            with open(self.json_file, 'r') as f:
                self.objects_data = json.load(f)
                self._init_saved_positions_from_data()
        else:
            self.objects_data = []
        
        # Statistics
        self.detection_count = 0
        self.start_time = None
        
        self.get_logger().info('Damaged Object Tracker Server initialized')
    
    def start_stop_callback(self, request, response):
        """Service callback to start/stop inspection"""
        with self.inspection_lock:
            if request.data and not self.inspection_active:
                # Start inspection
                self._start_inspection()
                response.success = True
                response.message = "Inspection started successfully"
                self.get_logger().info('Inspection started')
                
            elif not request.data and self.inspection_active:
                # Stop inspection
                self._stop_inspection()
                response.success = True
                response.message = "Inspection stopped successfully"
                self.get_logger().info('Inspection stopped')
                
            elif request.data and self.inspection_active:
                response.success = False
                response.message = "Inspection already running"
                
            else:
                response.success = False
                response.message = "Inspection already stopped"
        
        return response
    
    def status_callback(self, request, response):
        """Service callback to get inspection status"""
        with self.inspection_lock:
            if self.inspection_active:
                elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                response.success = True
                response.message = (
                    f"Inspection ACTIVE - Running for {elapsed_time:.1f}s, "
                    f"Detected {self.detection_count} objects, "
                    f"Saved {len(self.saved_ids)} unique objects"
                )
            else:
                response.success = False
                response.message = (
                    f"Inspection INACTIVE - Total saved objects: {len(self.saved_ids)}"
                )
        
        return response
    
    def _start_inspection(self):
        """Start the inspection process"""
        self.inspection_active = True
        self.start_time = self.get_clock().now()
        self.detection_count = 0
        
        # Reset tracking state
        self.last_gray = None
        self.tracker = DeepSort(
            max_age=30,
            n_init=3,
            max_cosine_distance=0.2
        )
        
        # Create subscriptions
        self.robot_odom_sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.save_robot_odom,
            10,
            callback_group=self.callback_group
        )
        
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.save_damaged_image,
            10,
            callback_group=self.callback_group
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.servo_sub = self.create_subscription(
            String,
            '/servo_angles',
            self.servo_callback,
            10,
            callback_group=self.callback_group
        )
    
    def _stop_inspection(self):
        """Stop the inspection process"""
        self.inspection_active = False
        
        # Destroy subscriptions
        if self.robot_odom_sub:
            self.destroy_subscription(self.robot_odom_sub)
            self.robot_odom_sub = None
        
        if self.camera_sub:
            self.destroy_subscription(self.camera_sub)
            self.camera_sub = None
        
        if self.depth_sub:
            self.destroy_subscription(self.depth_sub)
            self.depth_sub = None
        
        if self.servo_sub:
            self.destroy_subscription(self.servo_sub)
            self.servo_sub = None
    
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
                
                obj_id = obj.get('id')
                if obj_id is not None:
                    self.saved_ids.add(obj_id)
    
    def is_close_to_existing_object(self, class_name, position):
        """Check if a detected object is too close to an existing saved object"""
        if position is None or class_name not in self.saved_object_positions:
            return False
            
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
        if not self.inspection_active:
            return
            
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {str(e)}")
    
    def servo_callback(self, msg):
        """Store the servo angles from camera_servo package"""
        if not self.inspection_active:
            return
            
        try:
            # Parse servo angles from string format "1000 2007"
            angles = msg.data.strip().split()
            if len(angles) >= 2:
                # Convert servo values to radians
                # Tilt: 2000 = 0°, 1000 = +90° up, 3000 = -90° down
                # Pan: 2000 = 0°, 1000 = -90° left, 3000 = +90° right
                tilt_servo = int(angles[0])  # vertical axis (up/down)
                pan_servo = int(angles[1])   # horizontal axis (left/right)
                
                # Convert to radians: 1000 servo units = 90° (π/2 radians)
                self.servo_angles['tilt'] = (2000 - tilt_servo) * (math.pi/2) / 1000
                self.servo_angles['pan'] = (pan_servo - 2000) * (math.pi/2) / 1000
                
                self.get_logger().debug(f"Servo angles - Tilt: {self.servo_angles['tilt']:.2f}rad, Pan: {self.servo_angles['pan']:.2f}rad")
        except Exception as e:
            self.get_logger().error(f"Error processing servo angles: {str(e)}")
    
    def save_robot_odom(self, msg):
        """Callback function that gets called when a new TF message is received"""
        if not self.inspection_active:
            return
            
        try:
            if msg.transforms:
                x = msg.transforms[0].transform.translation.x
                y = msg.transforms[0].transform.translation.y
                z = msg.transforms[0].transform.translation.z
                self.robot_position = (x, y, z)
                self.get_logger().debug(f"Robot position: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        except (IndexError, Exception) as e:
            self.get_logger().error(f"Error processing TF message: {str(e)}")
    
    def get_3d_position(self, bbox, depth_img):
        """Calculate 3D position of an object given its bounding box and depth image"""
        if depth_img is None:
            self.get_logger().warn("No depth image available")
            return None
        
        x1, y1, x2, y2 = bbox
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        
        height, width = depth_img.shape
        if not (0 <= center_x < width and 0 <= center_y < height):
            self.get_logger().warn(f"Invalid bbox center: ({center_x}, {center_y})")
            return None
        
        # Get depth at center point
        region_size = 5
        x_min = max(0, center_x - region_size)
        x_max = min(width, center_x + region_size)
        y_min = max(0, center_y - region_size)
        y_max = min(height, center_y + region_size)
        
        depth_region = depth_img[y_min:y_max, x_min:x_max]
        valid_depths = depth_region[depth_region > 0]
        
        if len(valid_depths) == 0:
            self.get_logger().warn("No valid depth values in region")
            return None
        
        depth_m = np.median(valid_depths) / 1000.0
        
        # Calculate 3D coordinates in camera frame
        x = (center_x - self.cx) * depth_m / self.fx
        y = (center_y - self.cy) * depth_m / self.fy
        z = depth_m
        
        # Transform to world frame using servo angles
        if 'pan' in self.servo_angles and 'tilt' in self.servo_angles:
            pan = self.servo_angles['pan']
            tilt = self.servo_angles['tilt']
            
            # Apply tilt and pan transformations
            y_tilt = y * math.cos(tilt) - z * math.sin(tilt)
            z_tilt = y * math.sin(tilt) + z * math.cos(tilt)
            
            x_pan = x * math.cos(pan) + z_tilt * math.sin(pan)
            z_pan = -x * math.sin(pan) + z_tilt * math.cos(pan)
            
            world_x = x_pan + self.robot_position[0]
            world_y = y_tilt + self.robot_position[1]
            world_z = z_pan + self.robot_position[2]
            
            return (world_x, world_y, world_z)
        else:
            return (x, y, z)
    
    def save_damaged_image(self, msg):
        """Process camera images and track damaged objects"""
        if not self.inspection_active:
            return
            
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Check if stationary
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if self.last_gray is None:
                self.last_gray = gray
                return
                
            diff = cv2.absdiff(gray, self.last_gray)
            moving_pixels = np.count_nonzero(diff > self.DIFF_THRESH)
            self.last_gray = gray
            
            if moving_pixels >= self.STATIONARY_THRESH:
                self.get_logger().debug("Moving - waiting for stability")
                return
            
            # Run YOLO detection
            results = self.model(frame)[0]
            xyxy = results.boxes.xyxy.cpu().numpy()
            scores = results.boxes.conf.cpu().numpy()
            classes = results.boxes.cls.cpu().numpy()
            
            # Convert to DeepSORT format
            raw_dets = []
            for (x1, y1, x2, y2), conf, cls in zip(xyxy, scores, classes):
                if conf < self.CONF_THRESH:
                    continue
                w = x2 - x1
                h = y2 - y1
                bbox_xywh = [float(x1), float(y1), float(w), float(h)]
                raw_dets.append((bbox_xywh, float(conf), int(cls)))
            
            # Update tracker
            tracks = self.tracker.update_tracks(raw_dets, frame=frame)
            
            # Process tracks
            detected_objects = []
            for t in tracks:
                if not t.is_confirmed():
                    continue

                tid = t.track_id
                self.unique_ids.add(tid)

                x1, y1, x2, y2 = map(int, t.to_ltrb())
                cls_id = t.det_class
                class_name = self.model.names.get(cls_id, str(cls_id))

                object_position = self.get_3d_position((x1, y1, x2, y2), self.depth_image)
                
                is_duplicate = False
                if object_position:
                    is_duplicate = self.is_close_to_existing_object(class_name, object_position)

                is_new_object = tid not in self.saved_ids and not is_duplicate

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
                
                # Draw visualization
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                if is_duplicate:
                    color = (0, 165, 255)
                    status = "DUPLICATE"
                elif is_new_object:
                    color = (0, 0, 255)
                    status = "NEW"
                else:
                    color = (0, 255, 0)
                    status = "KNOWN"
                    
                text = f"{class_name} ID:{tid} ({status})"
                
                if object_position:
                    x, y, z = object_position
                    text += f" | Pos: ({x:.2f}, {y:.2f}, {z:.2f})"
                    
                cv2.putText(frame, text, (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # Update detection count
            self.detection_count += len(detected_objects)
            
            # Save new objects
            if detected_objects:
                new_objects_count = sum(1 for obj in detected_objects if obj['is_new'])
                duplicate_count = sum(1 for obj in detected_objects if obj['is_duplicate'])
                self.get_logger().info(
                    f"Tracking {len(detected_objects)} objects, {new_objects_count} new, "
                    f"{duplicate_count} duplicates, {len(self.unique_ids)} unique IDs total"
                )
                self.save_damaged_object_poses(detected_objects, frame)
            
            # Add status overlay
            cv2.putText(frame, f"INSPECTION: {'ACTIVE' if self.inspection_active else 'INACTIVE'}", 
                       (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.putText(frame, f"Unique objects: {len(self.unique_ids)}", (20, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
            cv2.putText(frame, f"Saved objects: {len(self.saved_ids)}", (20, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            
            # Publish annotated image
            annotated_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
    
    def save_damaged_object_poses(self, detected_objects, frame):
        """Save new detected objects' images and poses to a JSON file"""
        new_objects = [obj for obj in detected_objects if obj['is_new']]
        
        if not new_objects:
            return
        
        self.get_logger().info(f"Saving {len(new_objects)} new damaged objects")
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        for obj in new_objects:
            obj_id = obj['id']
            x1, y1, x2, y2 = obj['bbox']
            class_name = obj.get('class_name')
            object_position = obj.get('object_position')
            
            if obj_id in self.saved_ids or obj.get('is_duplicate', False):
                continue
                
            self.saved_ids.add(obj_id)
            
            if object_position and class_name:
                if class_name not in self.saved_object_positions:
                    self.saved_object_positions[class_name] = []
                self.saved_object_positions[class_name].append(object_position)
            
            # Save full frame instead of cropped object
            obj_img = frame
            if obj_img.size == 0:
                self.get_logger().warn(f"Empty region for object ID {obj_id}, skipping")
                continue
                
            img_filename = f"object_{obj_id}_{timestamp}.jpg"
            img_path = os.path.join(self.output_dir, img_filename)
            cv2.imwrite(img_path, obj_img)
            
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
            
            if object_position:
                x, y, z = object_position
                object_data['position'] = {
                    'x': x,
                    'y': y,
                    'z': z
                }
            else:
                object_data['robot_position'] = {
                    'x': obj['robot_position'][0],
                    'y': obj['robot_position'][1],
                    'z': obj['robot_position'][2]
                }
            
            self.objects_data.append(object_data)
            self.get_logger().info(f"Saved object ID {obj_id} to {img_path}")
        
        # Save updated JSON file
        with open(self.json_file, 'w') as f:
            json.dump(self.objects_data, f, indent=2)


def main(args=None):
    rclpy.init(args=args)
    
    tracker_server = DamagedObjectTrackerServer()
    
    # Use MultiThreadedExecutor for concurrent processing
    executor = MultiThreadedExecutor()
    executor.add_node(tracker_server)
    
    try:
        tracker_server.get_logger().info('Starting Damaged Object Tracker Server...')
        executor.spin()
    except KeyboardInterrupt:
        tracker_server.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        executor.shutdown()
        tracker_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()