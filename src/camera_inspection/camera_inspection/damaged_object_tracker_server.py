import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import json
import os
from datetime import datetime
import math
from std_srvs.srv import SetBool, Trigger
from threading import Lock
from go2_interfaces.action import DetectClosest
from std_msgs.msg import String as StringMsg

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
        # Publisher to move camera servos (absolute namespace to reach camera_servo node)
        self.servo_move_pub = self.create_publisher(StringMsg, '/servo_move', 10)
        
        # Initialize CV bridge and YOLO model
        self.cv_bridge = CvBridge()
        self.model = YOLO('src/camera_inspection/model/damaged_object_yolo.pt')

        # Warmup YOLO model to avoid slow first inference
        dummy_frame = np.zeros((640, 640, 3), dtype=np.uint8)
        _ = self.model(dummy_frame, verbose=False)
        self.get_logger().info('YOLO model warmed up and ready')

        # Create a reusable Rate object for servo sequence timing (10 Hz = 100ms)
        self.sequence_rate = self.create_rate(10)

        # Object tracking parameters (configurable via ROS params)
        # Defaults chosen for moderate sensitivity; can be overridden at runtime
        self.declare_parameter('conf_thresh', 0.5)
        self.declare_parameter('frame_stationary_thresh', 3000)  # moving pixels threshold
        self.declare_parameter('frame_diff_thresh', 20)          # pixel intensity diff threshold
        self.declare_parameter('proximity_thresh_m', 0.3)
        self.declare_parameter('servo_motion_thresh_rad', 0.02)  # ~1.1 degrees
        self.declare_parameter('servo_stable_frames', 5)
        self.declare_parameter('disable_motion_gating', False)

        self.CONF_THRESH = float(self.get_parameter('conf_thresh').value)
        self.STATIONARY_THRESH = int(self.get_parameter('frame_stationary_thresh').value)
        self.DIFF_THRESH = int(self.get_parameter('frame_diff_thresh').value)
        self.PROXIMITY_THRESH = float(self.get_parameter('proximity_thresh_m').value)

        # Camera motion detection parameters
        self.SERVO_MOTION_THRESH = float(self.get_parameter('servo_motion_thresh_rad').value)
        self.SERVO_STABLE_FRAMES = int(self.get_parameter('servo_stable_frames').value)
        self.disable_motion_gating = bool(self.get_parameter('disable_motion_gating').value)
        
        # Part 2: DeepSORT removed - only save during DetectClosest sessions
        # No background tracking/saving in normal mode
        
        # Tracking state
        self.unique_ids = set()
        self.saved_ids = set()
        self.last_gray = None
        self.robot_position = (0, 0, 0)
        
        # Depth and servo state
        self.depth_image = None
        self.servo_angles = {}
        self.previous_servo_angles = {}
        self.color_image_shape = None  # (height, width)
        self.depth_image_shape = None  # (height, width)
        
        # Motion detection state
        self.servo_motion_detected = False
        self.frames_since_servo_stopped = 0
        self.camera_is_moving = False
        
        # Store positions of saved objects by class for proximity check
        self.saved_object_positions = {}
        
        # Camera intrinsic parameters for RealSense D435i
        # These should match the depth camera calibration
        # Typical values for 640x480 depth resolution
        self.fx = 386.738  # Focal length x
        self.fy = 386.738  # Focal length y  
        self.cx = 320.0    # Principal point x (image center)
        self.cy = 240.0    # Principal point y (image center)
        
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

        # Parameters: two class groups and their target servo angles (tilt, pan)
        # Classes are provided as comma-separated string parameters for simplicity.
        self.declare_parameter('group_a_classes', '')
        self.declare_parameter('group_b_classes', '')
        self.declare_parameter('group_a_servo_tilt', 2000)
        self.declare_parameter('group_a_servo_pan', 2000)
        self.declare_parameter('group_b_servo_tilt', 2000)
        self.declare_parameter('group_b_servo_pan', 2000)
        # Optional: per-class and per-group multi-step camera motion sequences encoded as JSON
        # Examples provided in detect_closest_params.yaml under camera_sequences_json and group_*_sequence_json
        self.declare_parameter('camera_sequences_json', '')
        self.declare_parameter('camera_sequence_repeat', False)
        self.declare_parameter('group_a_sequence_json', '')
        self.declare_parameter('group_b_sequence_json', '')

        self._load_class_servo_params()
        self._load_camera_sequences()

        # Action server: DetectClosest (find one closest-to-center of a class)
        self.detect_action_server = ActionServer(
            self,
            DetectClosest,
            '~/detect_closest',
            execute_callback=self.execute_detect_closest,
            goal_callback=self.detect_goal_callback,
            cancel_callback=self.detect_cancel_callback,
            callback_group=self.callback_group,
        )

        # Session state for DetectClosest
        self.session_active = False
        self.session_target_id = None
        self.session_target_class = None
        self.session_require_depth = False
        self.session_confirm_frames = 3
        self.session_deadline = None
        self.session_started_inspection = False
        self.session_saved = False
        self.session_closest_track_id = None
        self.session_closest_stable_frames = 0
        self.session_candidates = 0
        self.session_closest_px_dist = float('inf')
        self.session_depth_ready = False
        self.session_result_info = None
        self.current_goal_handle = None
        
        # Timer-based servo sequence state machine
        self.sequence_state = 'IDLE'  # States: IDLE, MOVING_TO_POSE, WAITING_AT_POSE, POST_SEQUENCE_TIMEOUT, COMPLETE
        self.sequence_steps = []
        self.sequence_step_index = 0
        self.sequence_step_start_time = None
        self.sequence_pose_start_time = None
        self.sequence_timer = None
        self.sequence_class_name = None
        self.sequence_timer_period = 0.1  # 10 Hz timer
        
        self.get_logger().info('Damaged Object Tracker Server initialized')

    def _load_class_servo_params(self):
        """Load class grouping and servo targets from parameters"""
        a_classes = self.get_parameter('group_a_classes').get_parameter_value().string_value
        b_classes = self.get_parameter('group_b_classes').get_parameter_value().string_value
        self.group_a_classes = set([c.strip() for c in a_classes.split(',') if c.strip()])
        self.group_b_classes = set([c.strip() for c in b_classes.split(',') if c.strip()])
        self.group_a_servo = (
            int(self.get_parameter('group_a_servo_tilt').value),
            int(self.get_parameter('group_a_servo_pan').value),
        )
        self.group_b_servo = (
            int(self.get_parameter('group_b_servo_tilt').value),
            int(self.get_parameter('group_b_servo_pan').value),
        )
        if self.group_a_classes:
            self.get_logger().info(f"Group A: {sorted(self.group_a_classes)} -> servo {self.group_a_servo}")
        if self.group_b_classes:
            self.get_logger().info(f"Group B: {sorted(self.group_b_classes)} -> servo {self.group_b_servo}")

    def _load_camera_sequences(self):
        """Load optional per-class and per-group motion sequences from parameters.

        Format (JSON string):
          {
            "crack": [
              {"tilt": 2100, "pan": 2000, "wait_s": 2.0},
              {"tilt": 2000, "pan": 1800, "wait_s": 2.0},
              {"tilt": 2200, "pan": 2200, "wait_s": 2.0}
            ],
            "light_on": [ ... ]
          }
        """
        self.camera_sequences = {}
        self.group_sequences = {'A': None, 'B': None}
        self.sequence_repeat = bool(self.get_parameter('camera_sequence_repeat').value)

        def parse_steps_list(steps, label):
            if not isinstance(steps, list):
                self.get_logger().warn(f"Sequence for {label} ignored: not a list")
                return None
            norm_steps = []
            for i, st in enumerate(steps):
                if not isinstance(st, dict):
                    self.get_logger().warn(f"Step {i} for {label} ignored: not an object")
                    continue
                try:
                    tilt = int(st.get('tilt'))
                    pan = int(st.get('pan'))
                    wait_s = float(st.get('wait_s', 1.0))
                    norm_steps.append({'tilt': tilt, 'pan': pan, 'wait_s': wait_s})
                except Exception:
                    self.get_logger().warn(f"Invalid fields in step {i} for {label}")
            return norm_steps if norm_steps else None

        # Per-class mapping
        raw_class = self.get_parameter('camera_sequences_json').get_parameter_value().string_value
        if raw_class:
            try:
                parsed = json.loads(raw_class)
                if not isinstance(parsed, dict):
                    self.get_logger().warn('camera_sequences_json must be a JSON object mapping class->steps[]')
                else:
                    for cls, steps in parsed.items():
                        norm_steps = parse_steps_list(steps, f"class '{cls}'")
                        if norm_steps:
                            self.camera_sequences[cls] = norm_steps
                    if self.camera_sequences:
                        classes = ', '.join(sorted(self.camera_sequences.keys()))
                        self.get_logger().info(f"Loaded camera sequences for classes: {classes} (repeat={self.sequence_repeat})")
            except Exception as e:
                self.get_logger().warn(f"Failed to parse camera_sequences_json: {e}")

        # Per-group sequences
        raw_ga = self.get_parameter('group_a_sequence_json').get_parameter_value().string_value
        if raw_ga:
            try:
                steps = json.loads(raw_ga)
                norm = parse_steps_list(steps, 'group A')
                if norm:
                    self.group_sequences['A'] = norm
                    self.get_logger().info("Loaded camera sequence for Group A")
            except Exception as e:
                self.get_logger().warn(f"Failed to parse group_a_sequence_json: {e}")

        raw_gb = self.get_parameter('group_b_sequence_json').get_parameter_value().string_value
        if raw_gb:
            try:
                steps = json.loads(raw_gb)
                norm = parse_steps_list(steps, 'group B')
                if norm:
                    self.group_sequences['B'] = norm
                    self.get_logger().info("Loaded camera sequence for Group B")
            except Exception as e:
                self.get_logger().warn(f"Failed to parse group_b_sequence_json: {e}")

    def _sequence_for_class(self, class_name):
        """Return steps list to use for a given class.

        Order of precedence: per-class > per-group > None.
        """
        if class_name in self.camera_sequences:
            return self.camera_sequences[class_name]
        if class_name in self.group_a_classes and self.group_sequences['A'] is not None:
            return self.group_sequences['A']
        if class_name in self.group_b_classes and self.group_sequences['B'] is not None:
            return self.group_sequences['B']
        return None

    @staticmethod
    def _extract_class_from_target_id(target_id):
        parts = target_id.split(':')
        if len(parts) != 3:
            raise ValueError("target_id must follow '<prefix>:<class>:<suffix>' format")
        if any(len(part.strip()) == 0 for part in parts):
            raise ValueError("target_id segments must be non-empty")
        return parts[1].strip()

    def set_camera_for_class(self, class_name):
        """Command servos to a predefined angle depending on class group.

        Returns True if a command was sent, False otherwise.
        """
        target = None
        if class_name in self.group_a_classes:
            target = self.group_a_servo
            group = 'A'
        elif class_name in self.group_b_classes:
            target = self.group_b_servo
            group = 'B'
        else:
            return False

        tilt, pan = target
        try:
            msg = StringMsg()
            msg.data = f"{tilt} {pan}"
            self.servo_move_pub.publish(msg)
            self.get_logger().info(f"Aligned camera for class '{class_name}' (Group {group}) -> tilt={tilt}, pan={pan}")
            return True
        except Exception as e:
            self.get_logger().warn(f"Failed to publish servo move for class '{class_name}': {e}")
            return False

    def wait_for_camera_stable(self, max_wait_s=3.0):
        """Wait briefly for servo motion to settle using existing motion gating state.

        Uses non-blocking Rate.sleep() to allow camera callbacks to execute during wait.
        """
        start = self.get_clock().now().nanoseconds / 1e9

        # Initial settle time - use Rate.sleep to yield to executor
        # This allows camera/depth/servo callbacks to run
        settle_cycles = int(0.2 / 0.1)  # 0.2s at 10Hz = 2 cycles
        for _ in range(settle_cycles):
            try:
                self.sequence_rate.sleep()
            except Exception:
                break

        # Wait for motion to stop, checking at 10 Hz
        while (self.servo_motion_detected or self.camera_is_moving):
            now = self.get_clock().now().nanoseconds / 1e9
            if now - start >= max_wait_s:
                self.get_logger().debug(f"Camera settle timeout after {max_wait_s}s")
                break
            # Yield to executor - allows callbacks to process
            try:
                self.sequence_rate.sleep()
            except Exception:
                break

    def _sequence_timer_callback(self):
        """Non-blocking timer callback to advance servo sequence state machine.
        
        This runs at 10 Hz and manages state transitions:
        IDLE → MOVING_TO_POSE → WAITING_AT_POSE → (repeat or finish) → POST_SEQUENCE_TIMEOUT → COMPLETE
        """
        now_s = self.get_clock().now().nanoseconds / 1e9
        
        # Check for early exit conditions (cancel request or object found)
        should_exit = False
        if self.current_goal_handle and self.current_goal_handle.is_cancel_requested:
            should_exit = True
            self.get_logger().info("Sequence cancelled by client")
        
        # State: MOVING_TO_POSE - Waiting for camera to stabilize after servo command
        if self.sequence_state == 'MOVING_TO_POSE':
            # Check if camera has stabilized
            if not self.servo_motion_detected and not self.camera_is_moving:
                # Camera is stable, transition to waiting at pose
                self.sequence_state = 'WAITING_AT_POSE'
                self.sequence_pose_start_time = now_s
                self.get_logger().info(f"Camera stabilized at pose {self.sequence_step_index + 1}/{len(self.sequence_steps)}")
            else:
                # Still moving, check for timeout
                elapsed = now_s - self.sequence_step_start_time
                if elapsed > 3.0:  # Max 3s to stabilize
                    self.get_logger().warn(f"Camera stabilization timeout after {elapsed:.1f}s, proceeding anyway")
                    self.sequence_state = 'WAITING_AT_POSE'
                    self.sequence_pose_start_time = now_s
        
        # State: WAITING_AT_POSE - Collecting detections at current pose
        elif self.sequence_state == 'WAITING_AT_POSE':
            current_step = self.sequence_steps[self.sequence_step_index]
            wait_s = current_step['wait_s']
            elapsed = now_s - self.sequence_pose_start_time
            
            # Publish feedback while waiting
            try:
                if self.current_goal_handle:
                    fb = DetectClosest.Feedback()
                    fb.active_class = self.sequence_class_name
                    fb.closest_px_dist = float(self.session_closest_px_dist if self.session_closest_px_dist != float('inf') else -1.0)
                    fb.candidates = int(self.session_candidates)
                    fb.camera_moving = bool(self.camera_is_moving)
                    fb.depth_ready = bool(self.depth_image is not None)
                    self.current_goal_handle.publish_feedback(fb)
            except Exception:
                pass
            
            # Check if we should move to next pose
            if should_exit or elapsed >= wait_s:
                self.sequence_step_index += 1
                
                # Check if more poses remain
                if self.sequence_step_index < len(self.sequence_steps) and not should_exit:
                    # Move to next pose
                    next_step = self.sequence_steps[self.sequence_step_index]
                    try:
                        msg = StringMsg()
                        msg.data = f"{next_step['tilt']} {next_step['pan']}"
                        self.servo_move_pub.publish(msg)
                        self.get_logger().info(
                            f"Sequence move {self.sequence_step_index + 1}/{len(self.sequence_steps)} "
                            f"for '{self.sequence_class_name}': tilt={next_step['tilt']}, "
                            f"pan={next_step['pan']} (wait {next_step['wait_s']:.2f}s)"
                        )
                        self.sequence_state = 'MOVING_TO_POSE'
                        self.sequence_step_start_time = now_s
                    except Exception as e:
                        self.get_logger().warn(f"Failed to publish sequence servo move: {e}")
                        # Skip to next step on error
                        self.sequence_step_index += 1
                else:
                    # No more poses or cancelled, transition to post-sequence timeout
                    self.get_logger().info(f"Servo sequence complete for class '{self.sequence_class_name}'")
                    self.sequence_state = 'POST_SEQUENCE_TIMEOUT'
                    self.sequence_pose_start_time = now_s
        
        # State: POST_SEQUENCE_TIMEOUT - 15-second grace period to process queued frames
        elif self.sequence_state == 'POST_SEQUENCE_TIMEOUT':
            elapsed = now_s - self.sequence_pose_start_time
            
            # Exit early if object was saved
            if self.session_saved:
                self.get_logger().info(f"Object saved during post-sequence timeout (after {elapsed:.1f}s), completing")
                self.sequence_state = 'COMPLETE'
            elif elapsed >= 15.0:
                # Timeout complete
                if not self.session_saved:
                    self.get_logger().info("Post-sequence timeout complete, no object found")
                self.sequence_state = 'COMPLETE'
            elif should_exit:
                self.get_logger().info("Post-sequence timeout cancelled")
                self.sequence_state = 'COMPLETE'
        
        # State: COMPLETE - Cleanup timer
        elif self.sequence_state == 'COMPLETE':
            if self.sequence_timer:
                self.sequence_timer.cancel()
                self.sequence_timer = None
                self.get_logger().debug("Sequence timer stopped")

    def start_camera_sequence(self, class_name):
        """Start non-blocking camera sequence using timer-based state machine.
        
        This replaces the old blocking run_camera_sequence(). Instead of blocking,
        it initializes a timer that advances the sequence state machine at 10 Hz.
        
        Returns True if a sequence was started, False if none configured.
        """
        steps = self._sequence_for_class(class_name)
        if not steps:
            return False
        
        # Initialize state machine
        self.sequence_state = 'MOVING_TO_POSE'
        self.sequence_steps = steps
        self.sequence_step_index = 0
        self.sequence_class_name = class_name
        self.sequence_step_start_time = self.get_clock().now().nanoseconds / 1e9
        self.sequence_pose_start_time = None
        
        # Send first servo command
        first_step = steps[0]
        try:
            msg = StringMsg()
            msg.data = f"{first_step['tilt']} {first_step['pan']}"
            self.servo_move_pub.publish(msg)
            self.get_logger().info(
                f"Starting sequence for '{class_name}': {len(steps)} poses, "
                f"first move tilt={first_step['tilt']}, pan={first_step['pan']} "
                f"(wait {first_step['wait_s']:.2f}s)"
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to publish initial servo move: {e}")
            return False
        
        # Create and start timer (runs at 10 Hz)
        self.sequence_timer = self.create_timer(
            self.sequence_timer_period,
            self._sequence_timer_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info(f"Sequence timer started at {1.0/self.sequence_timer_period:.0f} Hz")
        return True

    def run_camera_sequence(self, class_name, deadline_s=None):
        """DEPRECATED: Legacy blocking implementation kept for compatibility.
        
        This is the old blocking version. Use start_camera_sequence() instead.
        Kept only for backwards compatibility - will be removed in future version.
        """
        self.get_logger().warn("run_camera_sequence() is deprecated, this shouldn't be called")
        return False
    
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
        self.servo_motion_detected = False
        self.frames_since_servo_stopped = 0
        self.camera_is_moving = False
        self.previous_servo_angles = {}
        
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

    # -------------------- Action: DetectClosest --------------------
    def detect_goal_callback(self, goal_request):
        if self.session_active:
            self.get_logger().warn('DetectClosest goal rejected: session already active')
            return GoalResponse.REJECT

        target_id = goal_request.target_id
        if not target_id:
            self.get_logger().warn('DetectClosest goal rejected: empty target_id')
            return GoalResponse.REJECT

        try:
            _ = self._extract_class_from_target_id(target_id)
        except ValueError as exc:
            self.get_logger().warn(f'DetectClosest goal rejected: invalid target_id "{target_id}": {exc}')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def detect_cancel_callback(self, goal_handle):
        self.get_logger().info('DetectClosest cancel requested')
        return CancelResponse.ACCEPT

    def _clear_session(self):
        self.session_active = False
        self.session_target_id = None
        self.session_target_class = None
        self.session_require_depth = False
        self.session_confirm_frames = 3
        self.session_deadline = None
        self.session_started_inspection = False
        self.session_saved = False
        self.session_closest_track_id = None
        self.session_closest_stable_frames = 0
        self.session_candidates = 0
        self.session_closest_px_dist = float('inf')
        self.session_depth_ready = False
        self.session_result_info = None
        self.current_goal_handle = None

    def execute_detect_closest(self, goal_handle):
        self.current_goal_handle = goal_handle
        goal = goal_handle.request

        # Extract parameters with defaults
        target_id = goal.target_id.strip()
        try:
            class_name = self._extract_class_from_target_id(target_id)
        except ValueError as exc:
            self.get_logger().error(f"DetectClosest goal aborted: invalid target_id '{target_id}': {exc}")
            result = DetectClosest.Result()
            result.success = False
            result.reason = 'invalid_target_id'
            result.saved_id = ''
            result.bbox_x1 = result.bbox_y1 = result.bbox_x2 = result.bbox_y2 = 0
            result.world_x = result.world_y = result.world_z = 0.0
            result.image_file = ''
            result.timestamp = 0.0
            goal_handle.abort()
            self.current_goal_handle = None
            return result

        timeout_s = goal.timeout_s if goal.timeout_s > 0.0 else 20.0
        confirm_frames = goal.confirm_frames if goal.confirm_frames > 0 else 1  # Default 1 for fast response with direct YOLO
        require_depth = bool(goal.require_depth)

        # Ensure inspection is active
        if not self.inspection_active:
            self._start_inspection()
            self.session_started_inspection = True

            # Wait for camera subscription to warm up and first frames to arrive
            # This ensures camera callbacks are active before servo sequence starts
            import time
            warmup_time = 1.0  # seconds
            self.get_logger().info(f"Waiting {warmup_time}s for camera warm-up...")
            time.sleep(warmup_time)
            self.get_logger().info("Camera warm-up complete")

        # Activate session
        self.session_active = True
        self.session_target_id = target_id
        self.session_target_class = class_name
        self.session_require_depth = require_depth
        self.session_confirm_frames = confirm_frames
        self.session_saved = False
        self.session_result_info = None
        self.get_logger().info(
            f"DetectClosest session started: target_id='{target_id}', class='{class_name}', timeout={timeout_s}s, confirm_frames={confirm_frames}, require_depth={require_depth}"
        )

        # Start non-blocking servo sequence using timer-based state machine
        # The sequence runs in parallel with camera callbacks (no more blocking!)
        ran_sequence = False
        try:
            ran_sequence = self.start_camera_sequence(class_name)
            if not ran_sequence:
                # No sequence configured, try single-pose alignment
                if self.set_camera_for_class(class_name):
                    self.wait_for_camera_stable(max_wait_s=3.0)
                # Manually set state to POST_SEQUENCE_TIMEOUT since no sequence is running
                self.sequence_state = 'POST_SEQUENCE_TIMEOUT'
                self.sequence_pose_start_time = self.get_clock().now().nanoseconds / 1e9
        except Exception as e:
            self.get_logger().warn(f"Failed to start servo sequence: {e}")
            self.sequence_state = 'COMPLETE'
        
        # Wait for sequence to complete (non-blocking polling at 10 Hz)
        # The actual work happens in _sequence_timer_callback() running in parallel
        if ran_sequence:
            self.get_logger().info("Waiting for timer-based sequence to complete...")
            while self.sequence_state != 'COMPLETE':
                # Check for cancel
                if self.current_goal_handle and self.current_goal_handle.is_cancel_requested:
                    self.get_logger().info("Action cancelled, stopping sequence")
                    self.sequence_state = 'COMPLETE'
                    if self.sequence_timer:
                        self.sequence_timer.cancel()
                        self.sequence_timer = None
                    break
                
                # Check for timeout (backup safety mechanism)
                # This should never trigger since timer handles timing internally
                
                # Sleep to yield to executor - allows timer and camera callbacks to run
                try:
                    self.sequence_rate.sleep()
                except Exception:
                    break
            
            self.get_logger().info(f"Sequence state machine completed for class '{class_name}'")
        else:
            # No sequence was started, wait for post-sequence timeout manually
            self.get_logger().info("No sequence configured, waiting 15s for detections...")
            timeout_cycles = int(15.0 / 0.1)
            for _ in range(timeout_cycles):
                if self.session_saved:
                    self.get_logger().info("Object saved, exiting early")
                    break
                if self.current_goal_handle and self.current_goal_handle.is_cancel_requested:
                    break
                try:
                    self.sequence_rate.sleep()
                except Exception:
                    break

        # Build result based on whether object was saved during sequence
        result = DetectClosest.Result()

        if self.session_saved and self.session_result_info is not None:
            # Success: object was found and saved during servo sequence
            info = self.session_result_info
            result.success = True
            result.reason = 'saved'
            result.saved_id = str(info.get('id', ''))
            x1, y1, x2, y2 = info.get('bbox', (0, 0, 0, 0))
            result.bbox_x1 = int(x1)
            result.bbox_y1 = int(y1)
            result.bbox_x2 = int(x2)
            result.bbox_y2 = int(y2)
            pos = info.get('world_position')
            if pos:
                result.world_x = float(pos[0])
                result.world_y = float(pos[1])
                result.world_z = float(pos[2])
            else:
                result.world_x = result.world_y = result.world_z = 0.0
            result.image_file = info.get('image_file', '')
            result.timestamp = float(info.get('timestamp', 0.0))

            goal_handle.succeed()
            self.get_logger().info(f"DetectClosest session completed: saved target_id '{result.saved_id}'")
        else:
            # Not found: no object detected during servo sequence
            result.success = False
            result.reason = 'not_found'
            result.saved_id = ''
            result.bbox_x1 = result.bbox_y1 = result.bbox_x2 = result.bbox_y2 = 0
            result.world_x = result.world_y = result.world_z = 0.0
            result.image_file = ''
            result.timestamp = 0.0

            goal_handle.succeed()  # Action succeeded (completed normally), but object not found
            self.get_logger().info(f'DetectClosest session completed: no object of class \'{class_name}\' found')

        # Cleanup - clear session state but keep inspection running to avoid destroying subscriptions mid-execution
        # Only stop inspection if we started it for this session AND we're in a safe state
        self._clear_session()

        # Stop inspection AFTER returning to avoid destroying handles while executor is using them
        if self.session_started_inspection:
            # Schedule inspection stop for next iteration to avoid InvalidHandle errors
            def delayed_stop():
                import time
                time.sleep(0.1)  # Small delay to let executor finish current iteration
                if self.inspection_active:
                    self._stop_inspection()

            import threading
            threading.Thread(target=delayed_stop, daemon=True).start()

        return result
    
    def _init_saved_positions_from_data(self):
        """Initialize the saved positions dictionary from loaded object data"""
        for obj in self.objects_data:
            class_name = obj.get('class_name')
            position = None
            
            # Handle new format (world_position + transformation_success)
            if 'world_position' in obj and obj.get('transformation_success', False):
                if obj['world_position'] is not None:
                    pos = obj['world_position']
                    position = (pos['x'], pos['y'], pos['z'])
            # Handle old format (position field)
            elif 'position' in obj:
                pos = obj['position']
                position = (pos['x'], pos['y'], pos['z'])
            # Skip records with only robot_position (no world coordinates)
            
            if class_name and position:
                if class_name not in self.saved_object_positions:
                    self.saved_object_positions[class_name] = []
                    
                self.saved_object_positions[class_name].append(position)
                
                obj_id = obj.get('id')
                if obj_id is not None:
                    self.saved_ids.add(str(obj_id))
    
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
            self.depth_image_shape = self.depth_image.shape[:2]  # (height, width)
            self.get_logger().debug(f"Depth image shape: {self.depth_image_shape}")
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {str(e)}")
    
    def servo_callback(self, msg):
        """Store the servo angles from camera_servo package and detect motion"""
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
                new_tilt = (2000 - tilt_servo) * (math.pi/2) / 1000
                new_pan = (pan_servo - 2000) * (math.pi/2) / 1000
                
                # Detect servo motion
                servo_motion = False
                if self.previous_servo_angles:
                    tilt_diff = abs(new_tilt - self.previous_servo_angles.get('tilt', new_tilt))
                    pan_diff = abs(new_pan - self.previous_servo_angles.get('pan', new_pan))
                    
                    if tilt_diff > self.SERVO_MOTION_THRESH or pan_diff > self.SERVO_MOTION_THRESH:
                        servo_motion = True
                        self.servo_motion_detected = True
                        self.frames_since_servo_stopped = 0
                        self.get_logger().debug(f"Servo motion detected - Tilt diff: {tilt_diff:.3f}rad, Pan diff: {pan_diff:.3f}rad")
                
                # Update servo angles
                self.previous_servo_angles = self.servo_angles.copy()
                self.servo_angles['tilt'] = new_tilt
                self.servo_angles['pan'] = new_pan
                
                # Update motion state
                if not servo_motion and self.servo_motion_detected:
                    self.frames_since_servo_stopped += 1
                    if self.frames_since_servo_stopped >= self.SERVO_STABLE_FRAMES:
                        self.servo_motion_detected = False
                        self.frames_since_servo_stopped = 0
                        self.get_logger().debug("Servo motion stopped, camera stabilized")
                
                self.get_logger().debug(f"Servo angles - Tilt: {self.servo_angles['tilt']:.2f}rad, Pan: {self.servo_angles['pan']:.2f}rad, Motion: {self.servo_motion_detected}")
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
    
    def get_3d_position(self, bbox, depth_img):
        """Calculate 3D position of an object given its bounding box and depth image"""
        if depth_img is None:
            self.get_logger().warn("No depth image available")
            return None
        
        x1, y1, x2, y2 = bbox
        depth_shape = depth_img.shape  # (height, width)
        
        # Get center of bounding box in color image coordinates
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        
        self.get_logger().debug(f"Original center: ({center_x}, {center_y})")
        
        # Scale center coordinates from color to depth image if resolutions differ
        if self.color_image_shape is not None:
            depth_center_x, depth_center_y = self.scale_coordinates(
                center_x, center_y, self.color_image_shape, depth_shape
            )
        else:
            depth_center_x, depth_center_y = center_x, center_y
            
        self.get_logger().debug(f"Scaled center: ({depth_center_x}, {depth_center_y})")
        
        # Verify coordinates are within depth image bounds
        height, width = depth_shape
        if not (0 <= depth_center_x < width and 0 <= depth_center_y < height):
            self.get_logger().warn(
                f"Invalid bbox center after scaling: color ({center_x}, {center_y}) → "
                f"depth ({depth_center_x}, {depth_center_y}), depth shape: {depth_shape}"
            )
            return None
        
        # Get depth at center point - average a small region for stability
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
            return None
        
        # Get median depth value in millimeters, convert to meters
        depth_mm = np.median(valid_depths)
        depth_m = depth_mm / 1000.0
        
        # Validate depth is within reasonable range (0.1m to 10m)
        if not (0.1 <= depth_m <= 10.0):
            self.get_logger().warn(f"Unreasonable depth value: {depth_m}m")
            return None
        
        # Calculate 3D coordinates in camera frame (ROS standard: X=right, Y=down, Z=forward)
        # Use the scaled coordinates for accurate projection
        cam_x = (depth_center_x - self.cx) * depth_m / self.fx  # Right (+) / Left (-)
        cam_y = (depth_center_y - self.cy) * depth_m / self.fy  # Down (+) / Up (-)
        cam_z = depth_m  # Forward distance from camera
        
        self.get_logger().debug(f"Camera frame position: X={cam_x:.3f}, Y={cam_y:.3f}, Z={cam_z:.3f}")
        
        # Transform to world frame using servo angles
        if 'pan' in self.servo_angles and 'tilt' in self.servo_angles:
            pan = self.servo_angles['pan']    # Horizontal rotation (left/right)
            tilt = self.servo_angles['tilt']  # Vertical rotation (up/down)
            
            self.get_logger().debug(f"Servo angles: Pan={pan:.3f}rad ({math.degrees(pan):.1f}°), Tilt={tilt:.3f}rad ({math.degrees(tilt):.1f}°)")
            
            # Step 1: Apply tilt rotation (around camera's X-axis)
            # Tilt rotates the Y-Z plane: positive tilt = look up, negative tilt = look down
            y_after_tilt = cam_y * math.cos(tilt) - cam_z * math.sin(tilt)
            z_after_tilt = cam_y * math.sin(tilt) + cam_z * math.cos(tilt)
            x_after_tilt = cam_x  # X unchanged by tilt
            
            # Step 2: Apply pan rotation (around robot's Z-axis / camera's Y-axis after tilt)
            # Pan rotates the X-Z plane: positive pan = look right, negative pan = look left
            # In world frame: X=forward, Y=left, Z=up when robot faces forward
            x_final = x_after_tilt * math.cos(pan) + z_after_tilt * math.sin(pan)
            y_final = -x_after_tilt * math.sin(pan) + z_after_tilt * math.cos(pan)  
            z_final = -y_after_tilt  # Convert camera Y (down) to world Z (up)
            
            self.get_logger().debug(f"After servo transforms: X={x_final:.3f}, Y={y_final:.3f}, Z={z_final:.3f}")
            
            # Step 3: Transform to world coordinates
            # Robot position gives us the robot's location in world frame
            # When robot is at home position facing forward: robot X-axis = world X-axis (forward)
            world_x = y_final + self.robot_position[0]  # Forward/backward in world
            world_y = -x_final + self.robot_position[1]  # Left/right in world  
            world_z = z_final + self.robot_position[2]   # Up/down in world
            
            self.get_logger().debug(f"World frame position: X={world_x:.3f}, Y={world_y:.3f}, Z={world_z:.3f}")
            
            return (world_x, world_y, world_z)
        else:
            # Fallback: return camera frame coordinates if no servo data
            self.get_logger().warn("No servo angle data available, returning camera frame coordinates")
            return (cam_z, -cam_x, -cam_y)  # Approximate transformation
    
    def save_damaged_image(self, msg):
        """Process camera images and track damaged objects"""
        if not self.inspection_active:
            return

        # Log callback execution during active session for debugging
        if self.session_active:
            self.get_logger().info(f"Camera callback executing during session (target: {self.session_target_class})")

        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.color_image_shape = frame.shape[:2]  # Store color image shape (height, width)
            
            # Check if camera is moving (frame difference + servo motion)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if self.last_gray is None:
                self.last_gray = gray
                if self.session_active:
                    self.get_logger().info("First frame - initializing motion detection (skipping YOLO)")
                return
                
            # Frame difference motion detection
            diff = cv2.absdiff(gray, self.last_gray)
            moving_pixels = np.count_nonzero(diff > self.DIFF_THRESH)
            frame_motion_detected = moving_pixels >= self.STATIONARY_THRESH
            
            # Combined motion detection: frame motion OR servo motion
            self.camera_is_moving = frame_motion_detected or self.servo_motion_detected
            
            self.last_gray = gray

            # Skip detection if camera is moving, UNLESS:
            # 1. Motion gating is disabled globally, OR
            # 2. We have an active session (must detect during servo sequence)
            skip_detection = self.camera_is_moving and not self.disable_motion_gating and not self.session_active

            if skip_detection:
                motion_reason = []
                if frame_motion_detected:
                    motion_reason.append(f"frame_motion({moving_pixels}px)")
                if self.servo_motion_detected:
                    motion_reason.append("servo_motion")

                log_msg = f"Camera moving - {', '.join(motion_reason)} - skipping detection"
                if self.session_active:
                    self.get_logger().warn(log_msg + f" (session active for '{self.session_target_class}')")
                else:
                    self.get_logger().debug(log_msg)
                return

            # Log motion state during active session
            if self.session_active:
                if self.camera_is_moving:
                    motion_parts = []
                    if frame_motion_detected:
                        motion_parts.append(f"frame({moving_pixels}px)")
                    if self.servo_motion_detected:
                        motion_parts.append("servo")
                    self.get_logger().info(f"Camera MOVING ({', '.join(motion_parts)}) - detecting anyway (session active)")
            
            # Run YOLO detection
            results = self.model(frame)[0]
            xyxy = results.boxes.xyxy.cpu().numpy()
            scores = results.boxes.conf.cpu().numpy()
            classes = results.boxes.cls.cpu().numpy()

            # Log raw detections during active session
            if self.session_active:
                if len(xyxy) > 0:
                    detected_class_names = [self.model.names.get(int(cls), str(cls)) for cls in classes]
                    self.get_logger().info(f"YOLO detected {len(xyxy)} objects: {detected_class_names} (scores: {[f'{s:.2f}' for s in scores]})")
                else:
                    self.get_logger().info(f"YOLO detected 0 objects (target: '{self.session_target_class}')")
            
            # DETECTION PROCESSING: Two different paths depending on mode
            detected_objects = []

            if self.session_active:
                # ===== SESSION MODE: Process YOLO detections directly (skip DeepSORT) =====
                # During DetectClosest, we don't need multi-frame tracking confirmation
                # Process detections immediately for faster response
                raw_count = len(xyxy)
                filtered_count = sum(1 for conf in scores if conf >= self.CONF_THRESH)
                self.get_logger().info(
                    f"Session mode: YOLO raw={raw_count}, after conf_thresh>={self.CONF_THRESH}: {filtered_count} "
                    f"(skipping DeepSORT)"
                )

                for (x1, y1, x2, y2), conf, cls in zip(xyxy, scores, classes):
                    if conf < self.CONF_THRESH:
                        continue

                    cls_id = int(cls)
                    class_name = self.model.names.get(cls_id, str(cls_id))

                    # Use a temporary tracking ID based on bbox (for frame-to-frame stability checks)
                    track_id = hash((int(x1), int(y1), int(x2), int(y2), cls_id)) % 100000

                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    object_position = self.get_3d_position((x1, y1, x2, y2), self.depth_image)

                    is_duplicate = False
                    if object_position:
                        is_duplicate = self.is_close_to_existing_object(class_name, object_position)

                    is_new_object = (self.session_target_id not in self.saved_ids) and not is_duplicate

                    object_data = {
                        'track_id': track_id,
                        'id': self.session_target_id,
                        'target_id': self.session_target_id,
                        'class_id': cls_id,
                        'class_name': class_name,
                        'bbox': (x1, y1, x2, y2),
                        'object_position': object_position,
                        'robot_position': self.robot_position,
                        'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                        'is_new': is_new_object,
                        'is_duplicate': is_duplicate,
                        'confidence': float(conf)
                    }
                    detected_objects.append(object_data)

                    # Draw visualization for session mode
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)  # Magenta for session mode
                    text = f"{class_name} conf:{conf:.2f}"
                    if object_position:
                        x, y, z = object_position
                        text += f" | ({x:.2f}, {y:.2f}, {z:.2f})"
                    cv2.putText(frame, text, (x1, y1-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

            else:
                # ===== NORMAL MODE: Just run YOLO for visualization, don't save anything =====
                # Part 2: Removed DeepSORT tracking and automatic saving
                # Server only saves when client explicitly triggers DetectClosest action
                for (x1, y1, x2, y2), conf, cls in zip(xyxy, scores, classes):
                    if conf < self.CONF_THRESH:
                        continue

                    cls_id = int(cls)
                    class_name = self.model.names.get(cls_id, str(cls_id))
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    # Draw visualization only - no tracking, no saving
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Green for detections
                    text = f"{class_name} conf:{conf:.2f}"
                    cv2.putText(frame, text, (x1, y1-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Update detection count (for session mode only)
            if self.session_active:
                self.detection_count += len(detected_objects)

            # Action session path: select closest candidate of target class and save once
            if self.session_active and self.session_target_class is not None:
                candidates = [o for o in detected_objects if o['class_name'] == self.session_target_class]
                self.session_candidates = len(candidates)
                self.session_depth_ready = self.depth_image is not None

                # Log all detected objects during session for debugging
                if detected_objects:
                    detected_classes = [o['class_name'] for o in detected_objects]
                    self.get_logger().info(
                        f"Session active, detected classes: {detected_classes}, "
                        f"target='{self.session_target_class}', candidates={len(candidates)}"
                    )

                if candidates:
                    h, w = frame.shape[:2]
                    cx_img, cy_img = w // 2, h // 2

                    def center_dist(obj):
                        x1, y1, x2, y2 = obj['bbox']
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2
                        return math.hypot(cx - cx_img, cy - cy_img)

                    # Respect require_depth; prefer non-duplicate; smallest distance
                    filtered = [o for o in candidates if (o['object_position'] is not None or not self.session_require_depth)]
                    if not filtered and not self.session_require_depth:
                        filtered = candidates

                    if filtered:
                        filtered.sort(key=lambda o: (o['is_duplicate'], center_dist(o)))
                        best = filtered[0]
                        self.session_closest_px_dist = center_dist(best)

                        # Stability check
                        if self.session_closest_track_id == best['track_id']:
                            self.session_closest_stable_frames += 1
                        else:
                            self.session_closest_track_id = best['track_id']
                            self.session_closest_stable_frames = 1

                        if (self.session_closest_stable_frames >= self.session_confirm_frames
                                and not self.session_saved and not best['is_duplicate']):
                            self.get_logger().info(
                                f"DetectClosest selecting target_id {best['id']} at {self.session_closest_px_dist:.1f}px for class '{self.session_target_class}'"
                            )
                            # Save only this object
                            self.save_damaged_object_poses([best], frame)
                            # Capture result info
                            img_file = ''
                            if hasattr(self, 'last_saved_object_info') and self.last_saved_object_info:
                                img_file = self.last_saved_object_info.get('image_file', '')
                            self.session_result_info = {
                                'id': best['id'],
                                'bbox': best['bbox'],
                                'world_position': best.get('object_position'),
                                'image_file': img_file,
                                'timestamp': best['timestamp'],
                            }
                            self.session_saved = True
            # Part 2: Removed automatic save logic in normal mode
            # Only save when session_active==True (client explicitly requested)
            
            # Add status overlay
            cv2.putText(frame, f"INSPECTION: {'ACTIVE' if self.inspection_active else 'INACTIVE'}", 
                       (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.putText(frame, f"CAMERA: {'MOVING' if self.camera_is_moving else 'STABLE'}", 
                       (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255) if self.camera_is_moving else (0, 255, 0), 2)
            cv2.putText(frame, f"Unique objects: {len(self.unique_ids)}", (20, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
            cv2.putText(frame, f"Saved objects: {len(self.saved_ids)}", (20, 150),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            
            # Add resolution information for debugging
            if self.color_image_shape and self.depth_image_shape:
                resolution_text = f"Color: {self.color_image_shape[1]}x{self.color_image_shape[0]}, Depth: {self.depth_image_shape[1]}x{self.depth_image_shape[0]}"
                cv2.putText(frame, resolution_text, (20, 190),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # Overlay active session info
            if self.session_active and self.session_target_class is not None:
                cv2.putText(frame, f"TARGET: {self.session_target_class}", (20, 230),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                if self.session_closest_px_dist != float('inf'):
                    cv2.putText(frame, f"Closest(px): {self.session_closest_px_dist:.1f}", (20, 270),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
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
            obj_id = str(obj['id'])
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
                
            safe_id = obj_id.replace(':', '_')
            img_filename = f"object_{safe_id}_{timestamp}.jpg"
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
            
            # Always save both robot position and world position
            object_data['robot_position'] = {
                'x': obj['robot_position'][0],
                'y': obj['robot_position'][1],
                'z': obj['robot_position'][2]
            }
            
            if object_position:
                x, y, z = object_position
                object_data['world_position'] = {
                    'x': x,
                    'y': y,
                    'z': z
                }
                object_data['transformation_success'] = True
            else:
                object_data['world_position'] = None
                object_data['transformation_success'] = False
            
            self.objects_data.append(object_data)
            self.get_logger().info(f"Saved object ID {obj_id} to {img_path}")

            # Cache info for action result
            try:
                self.last_saved_object_info = {
                    'id': obj_id,
                    'class_name': class_name,
                    'image_file': img_filename,
                    'timestamp': obj['timestamp'],
                    'bbox': (x1, y1, x2, y2),
                    'robot_position': obj['robot_position'],
                    'world_position': object_position if object_position else None,
                }
            except Exception:
                pass
        
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
