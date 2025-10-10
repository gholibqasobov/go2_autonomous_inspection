#!/usr/bin/env python3
"""
ROS2 Bridge for Go2 Web Interface
Connects Flask server to ROS2 topics for real robot communication
"""

import base64
import io
import math
import struct
import threading
import time
from typing import Optional, Dict, Callable, List, Tuple
from pathlib import Path

# Flask imports
from flask import Flask, jsonify, request, send_from_directory
from flask_cors import CORS
from PIL import Image as PILImage
import yaml

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from geometry_msgs.msg import PoseStamped, PointStamped, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image as ROSImage
    from sensor_msgs.msg import PointCloud2
    from tf2_msgs.msg import TFMessage
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS2 not available. Install rclpy and ROS2 packages to enable robot connection.")


class Go2ROS2Bridge:
    """Bridge between Flask web interface and ROS2 topics"""

    def __init__(
        self,
        enable_ros2: bool = True,
        robot_namespace: str = '',
        map_frame: str = 'map',
        robot_frame: str = 'base',
        tf_topic: str = '/tf',
        trajectory_topic: str = 'trajectory',
        camera_topic_sim: str = '/camera/image/raw',
        camera_topic_real: str = '/camera/img_raw',
        odom_topic: str = 'odom',
        map_name: str = 'cabinet_unity'
    ):
        """
        Initialize ROS2 bridge

        Args:
            enable_ros2: Enable ROS2 connection (requires ROS2 to be sourced)
            robot_namespace: Namespace for multi-robot support (e.g., 'robot0')
            map_name: Name of the map to load origin from
        """
        self.enabled = enable_ros2 and ROS2_AVAILABLE
        self.robot_namespace = robot_namespace
        self.map_frame = map_frame.lstrip('/')
        self.robot_frame = robot_frame.lstrip('/')
        self.tf_topic = tf_topic
        self.trajectory_topic = trajectory_topic
        self.camera_topic_sim = camera_topic_sim
        self.camera_topic_real = camera_topic_real
        self.odom_topic = odom_topic

        # Operating mode configuration
        self.current_mode = 'none'  # 'simulation', 'real', or 'none'

        # Map offset (applied to map, not robot)
        self.map_offset_x = 0.0
        self.map_offset_y = 0.0
        self.map_offset_theta = 0.0
        self._load_map_offset(map_name)

        # Robot state
        self.robot_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.position_lock = threading.Lock()
        self.last_pose_time = 0

        # Frame handling
        self.namespace_clean = robot_namespace.strip('/') if robot_namespace else ''
        self._map_frames = self._build_frame_candidates(self.map_frame)
        self._robot_frames = self._build_frame_candidates(self.robot_frame)

        # Trajectory state
        self.trajectory_points: List[Tuple[float, float]] = []
        self.trajectory_frame_id: str = self.map_frame
        self.trajectory_lock = threading.Lock()

        # Camera state
        self.camera_lock = threading.Lock()
        self.latest_image_bytes: Optional[bytes] = None
        self.latest_image_format: str = 'jpeg'
        self.latest_image_encoding: Optional[str] = None
        self.latest_image_timestamp: float = 0.0
        self.latest_image_size: Tuple[int, int] = (0, 0)

        # ROS2 node
        self.node: Optional['Go2Node'] = None
        self.ros_thread: Optional[threading.Thread] = None

        # Callbacks
        self.position_callback: Optional[Callable] = None

        if self.enabled:
            self._initialize_ros2()

    def _load_map_offset(self, map_name: str):
        """Load map offset from map YAML file"""
        try:
            # Import here to avoid circular dependency
            map_base_path = Path(__file__).resolve().parent / 'maps'
            yaml_path = map_base_path / f'{map_name}.yaml'

            if yaml_path.exists():
                with open(yaml_path, 'r') as f:
                    map_data = yaml.safe_load(f)
                    origin = map_data.get('origin', [0.0, 0.0, 0.0])
                    self.map_offset_x = float(origin[0])
                    self.map_offset_y = float(origin[1])
                    self.map_offset_theta = float(origin[2])
                    print(f"Loaded map offset for '{map_name}': [{self.map_offset_x}, {self.map_offset_y}, {self.map_offset_theta}]")
            else:
                print(f"Map file not found: {yaml_path}, using zero offset")
                self.map_offset_x = 0.0
                self.map_offset_y = 0.0
                self.map_offset_theta = 0.0
        except Exception as e:
            print(f"Failed to load map offset: {e}, using zero offset")
            self.map_offset_x = 0.0
            self.map_offset_y = 0.0
            self.map_offset_theta = 0.0

    def update_map_offset(self, map_name: str):
        """Update map offset when map is switched"""
        self._load_map_offset(map_name)

    def set_mode(self, mode: str):
        """
        Set the operating mode for the system

        Args:
            mode: Operating mode ('simulation', 'real', or 'none')
        """
        if mode not in ['simulation', 'real', 'none']:
            raise ValueError(f"Invalid mode: {mode}. Must be 'simulation', 'real', or 'none'")

        self.current_mode = mode
        print(f"Operating mode set to: {mode}")

    def get_mode(self) -> str:
        """
        Get the current operating mode

        Returns:
            Current mode string
        """
        return self.current_mode

    def _initialize_ros2(self):
        """Initialize ROS2 node and topics"""
        if not ROS2_AVAILABLE:
            print("ROS2 not available. Cannot initialize.")
            return

        try:
            # Initialize ROS2
            if not rclpy.ok():
                rclpy.init()

            # Create node
            self.node = Go2Node(
                namespace=self.robot_namespace,
                odom_callback=self._on_odometry_received,
                tf_callback=self._on_tf_message,
                trajectory_callback=self._on_trajectory_received,
                camera_callback=self._on_image_received,
                odom_topic=self.odom_topic,
                tf_topic=self.tf_topic,
                trajectory_topic=self.trajectory_topic,
                camera_topic_sim=self.camera_topic_sim,
                camera_topic_real=self.camera_topic_real
            )

            # Start ROS2 spin in separate thread
            self.ros_thread = threading.Thread(target=self._spin_ros2, daemon=True)
            self.ros_thread.start()

            print(f"ROS2 bridge initialized successfully (namespace: {self.robot_namespace or 'default'})")

        except Exception as e:
            print(f"Failed to initialize ROS2 bridge: {e}")
            self.enabled = False

    def _spin_ros2(self):
        """Spin ROS2 node in separate thread"""
        try:
            rclpy.spin(self.node)
        except Exception as e:
            print(f"ROS2 spin error: {e}")

    def _on_odometry_received(self, msg: 'Odometry'):
        """Callback when odometry message is received"""
        # Extract position from odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract orientation (quaternion to yaw)
        quat = msg.pose.pose.orientation
        theta = self._quaternion_to_yaw(quat.x, quat.y, quat.z, quat.w)

        # Store raw position (transformation will be applied in frontend)
        # Update position with thread safety
        with self.position_lock:
            self.robot_position = {
                'x': x,
                'y': y,
                'theta': theta
            }
            self.last_pose_time = time.time()

        # Trigger callback if set
        if self.position_callback:
            self.position_callback(self.robot_position)

    def _on_tf_message(self, msg: 'TFMessage'):
        """Handle incoming TF messages for pose estimation"""
        try:
            for transform in msg.transforms:
                parent = transform.header.frame_id.lstrip('/')
                child = transform.child_frame_id.lstrip('/')

                if parent not in self._map_frames or child not in self._robot_frames:
                    continue

                translation = transform.transform.translation
                rotation = transform.transform.rotation

                x = float(translation.x)
                y = float(translation.y)
                theta = self._quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)

                # Store raw position (transformation will be applied in frontend)
                with self.position_lock:
                    self.robot_position = {
                        'x': x,
                        'y': y,
                        'theta': theta
                    }
                    self.last_pose_time = time.time()

                if self.position_callback:
                    self.position_callback(self.robot_position)

        except Exception as exc:
            print(f"Failed to process TF message: {exc}")

    def _on_trajectory_received(self, msg: 'PointCloud2'):
        """Handle incoming trajectory updates"""
        try:
            points = self._pointcloud2_to_xy(msg)

            with self.trajectory_lock:
                self.trajectory_points = points
                self.trajectory_frame_id = msg.header.frame_id.lstrip('/') if msg.header.frame_id else self.map_frame

        except Exception as exc:
            print(f"Failed to process trajectory message: {exc}")

    def _on_image_received(self, msg: 'ROSImage', topic_name: str):
        """Handle incoming camera frames with mode-based filtering"""
        try:
            # Filter based on current mode and topic
            if self.current_mode == 'simulation' and topic_name != self.camera_topic_sim:
                return  # Ignore real camera in simulation mode
            elif self.current_mode == 'real' and topic_name != self.camera_topic_real:
                return  # Ignore simulation camera in real mode
            # In 'none' mode, accept either camera feed

            jpeg_bytes = self._image_msg_to_jpeg(msg)
            if jpeg_bytes is None:
                return

            with self.camera_lock:
                self.latest_image_bytes = jpeg_bytes
                self.latest_image_encoding = msg.encoding
                self.latest_image_format = 'jpeg'
                self.latest_image_timestamp = time.time()
                self.latest_image_size = (msg.width, msg.height)

        except Exception as exc:
            print(f"Failed to process camera image: {exc}")

    def _pointcloud2_to_xy(self, msg: 'PointCloud2') -> List[Tuple[float, float]]:
        """Convert PointCloud2 message to list of (x, y) tuples"""
        if msg.width * msg.height == 0 or msg.point_step == 0:
            return []

        x_offset = next((field.offset for field in msg.fields if field.name == 'x'), None)
        y_offset = next((field.offset for field in msg.fields if field.name == 'y'), None)

        if x_offset is None or y_offset is None:
            return []

        endian_prefix = '<' if not msg.is_bigendian else '>'

        raw_data = msg.data
        if isinstance(raw_data, memoryview):
            data_view = raw_data
        elif isinstance(raw_data, (bytes, bytearray)):
            data_view = memoryview(raw_data)
        else:
            data_view = memoryview(bytes(raw_data))

        total_points = msg.width * msg.height
        stride = max(1, total_points // 1000)  # Downsample to protect bandwidth

        points: List[Tuple[float, float]] = []
        for index in range(0, total_points, stride):
            offset = index * msg.point_step
            try:
                x = struct.unpack_from(f'{endian_prefix}f', data_view, offset + x_offset)[0]
                y = struct.unpack_from(f'{endian_prefix}f', data_view, offset + y_offset)[0]
            except struct.error:
                continue

            if math.isfinite(x) and math.isfinite(y):
                points.append((float(x), float(y)))

        return points

    def _image_msg_to_jpeg(self, msg: 'ROSImage') -> Optional[bytes]:
        """Convert a ROS2 Image message to JPEG bytes"""
        if msg.width == 0 or msg.height == 0:
            return None

        data = msg.data
        if isinstance(data, (bytes, bytearray)):
            raw_bytes = bytes(data)
        else:
            raw_bytes = bytes(data)

        encoding = (msg.encoding or '').lower()
        mode = None
        raw_mode = None

        if encoding == 'rgb8':
            mode = 'RGB'
            raw_mode = 'RGB'
        elif encoding == 'bgr8':
            mode = 'RGB'
            raw_mode = 'BGR'
        elif encoding == 'rgba8':
            mode = 'RGBA'
            raw_mode = 'RGBA'
        elif encoding == 'bgra8':
            mode = 'RGBA'
            raw_mode = 'BGRA'
        elif encoding in ('mono8', '8uc1'):
            mode = 'L'
            raw_mode = 'L'
        else:
            print(f"Unsupported image encoding received: {msg.encoding}")
            return None

        try:
            image = self._build_image_from_bytes(
                raw_bytes,
                mode,
                raw_mode,
                msg.width,
                msg.height
            )
        except Exception as exc:
            print(f"Failed to decode image bytes: {exc}")
            return None

        if image.mode not in ('RGB', 'L'):
            image = image.convert('RGB')

        buffer = io.BytesIO()
        image.save(buffer, format='JPEG', quality=80)
        return buffer.getvalue()

    @staticmethod
    def _build_image_from_bytes(data: bytes, mode: str, raw_mode: str, width: int, height: int):
        """Create a PIL image from raw bytes"""
        from PIL import Image

        return Image.frombytes(mode, (width, height), data, 'raw', raw_mode)

    def _build_frame_candidates(self, frame: str) -> set:
        """Build possible frame ids considering namespace variations"""
        frame_clean = frame.lstrip('/')
        if not frame_clean:
            return set()

        base_name = frame_clean.split('/')[-1]
        prefix = '' if '/' not in frame_clean else frame_clean[:-(len(base_name))]

        # Common aliases for frequently encountered frames
        robot_aliases = {'base', 'base_link', 'base_footprint', 'body', 'sensor'}
        map_aliases = {'map', 'world'}

        alias_pool = {base_name}
        if base_name in robot_aliases:
            alias_pool.update(robot_aliases)
        if base_name in map_aliases:
            alias_pool.update(map_aliases)

        candidates = set()
        for alias in alias_pool:
            candidates.add(alias)
            if prefix:
                candidates.add(f'{prefix}{alias}')

        if self.namespace_clean:
            namespace_prefix = f'{self.namespace_clean}/'
            candidates.update(f'{namespace_prefix}{alias}' for alias in list(candidates))

        return candidates

    @staticmethod
    def _quaternion_to_yaw(x, y, z, w):
        """Convert quaternion to yaw angle (radians)"""
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def apply_map_transformation(self, position: Dict[str, float]) -> Dict[str, float]:
        """
        Apply map offset transformation to a position

        Args:
            position: Dict with x, y, theta

        Returns:
            Transformed position dict with x, y, theta
        """
        x = position.get('x', 0.0)
        y = position.get('y', 0.0)
        theta = position.get('theta', 0.0)

        # Apply 2D transformation with rotation and translation
        cos_theta = math.cos(self.map_offset_theta)
        sin_theta = math.sin(self.map_offset_theta)
        x_offset = x * cos_theta - y * sin_theta + self.map_offset_x
        y_offset = x * sin_theta + y * cos_theta + self.map_offset_y
        theta_offset = theta + self.map_offset_theta

        return {
            'x': x_offset,
            'y': y_offset,
            'theta': theta_offset
        }

    def get_robot_position(self) -> Dict[str, float]:
        """
        Get current robot position

        Returns:
            Dict with x, y, theta
        """
        with self.position_lock:
            return self.robot_position.copy()

    def get_trajectory(self, prune_radius: Optional[float] = None) -> Dict[str, object]:
        """Return cached trajectory data, optionally pruning visited points"""

        robot_pose = self.get_robot_position()

        with self.trajectory_lock:
            should_prune = (
                prune_radius
                and prune_radius > 0
                and self.trajectory_points
                and (not self.enabled or self.last_pose_time > 0)
            )

            if should_prune:
                rx = robot_pose.get('x', 0.0)
                ry = robot_pose.get('y', 0.0)
                radius_sq = prune_radius * prune_radius
                filtered_points: List[Tuple[float, float]] = []
                removed_any = False

                for x, y in self.trajectory_points:
                    if ((x - rx) * (x - rx) + (y - ry) * (y - ry)) <= radius_sq:
                        removed_any = True
                        continue
                    filtered_points.append((x, y))

                if removed_any:
                    self.trajectory_points = filtered_points

            points_copy = list(self.trajectory_points)
            frame_id = self.trajectory_frame_id

        return {
            'frame_id': frame_id,
            'points': [{'x': x, 'y': y} for x, y in points_copy]
        }

    def send_navigation_goal(self, x: float, y: float, frame_id: str = 'map') -> bool:
        """
        Send navigation goal to robot based on current operating mode

        Args:
            x: Goal X coordinate (meters)
            y: Goal Y coordinate (meters)
            frame_id: Reference frame (default: 'map')

        Returns:
            Success status
        """
        if not self.enabled or not self.node:
            print("ROS2 bridge not enabled. Cannot send goal.")
            return False

        try:
            # Get current robot position for theta calculation
            robot_pos = self.get_robot_position()

            # Use the node's method to publish goal with current mode and robot position
            self.node.publish_goal(x, y, frame_id, self.current_mode, robot_pos)
            print(f"Navigation goal sent: ({x:.2f}, {y:.2f}) in frame '{frame_id}' [mode: {self.current_mode}]")
            return True

        except Exception as e:
            print(f"Failed to send navigation goal: {e}")
            return False

    def send_velocity_command(self, linear_x: float, linear_y: float, angular_z: float) -> bool:
        """
        Send velocity command to robot

        Args:
            linear_x: Linear velocity in X (m/s)
            linear_y: Linear velocity in Y (m/s)
            angular_z: Angular velocity (rad/s)

        Returns:
            Success status
        """
        if not self.enabled or not self.node:
            return False

        try:
            self.node.publish_cmd_vel(linear_x, linear_y, angular_z)
            return True
        except Exception as e:
            print(f"Failed to send velocity command: {e}")
            return False

    def is_connected(self) -> bool:
        """Check if ROS2 is connected and receiving data"""
        if not self.enabled:
            return False

        # Check if we've received recent odometry (within last 2 seconds)
        return (time.time() - self.last_pose_time) < 2.0

    def get_latest_camera_image(self, max_age: Optional[float] = None) -> Optional[Dict[str, object]]:
        """Return the most recent camera frame as base64 data"""
        with self.camera_lock:
            if not self.latest_image_bytes:
                return None

            if max_age is not None and (time.time() - self.latest_image_timestamp) > max_age:
                return None

            encoded = base64.b64encode(self.latest_image_bytes).decode('ascii')

            return {
                'data': encoded,
                'format': self.latest_image_format,
                'encoding': self.latest_image_encoding,
                'timestamp': self.latest_image_timestamp,
                'width': self.latest_image_size[0],
                'height': self.latest_image_size[1]
            }

    def shutdown(self):
        """Shutdown ROS2 bridge"""
        if self.node:
            self.node.destroy_node()

        if self.enabled and rclpy.ok():
            rclpy.shutdown()


if ROS2_AVAILABLE:

    class Go2Node(Node):
        """ROS2 node for Go2 robot communication"""

        def __init__(
            self,
            namespace: str = '',
            odom_callback: Optional[Callable] = None,
            tf_callback: Optional[Callable] = None,
            trajectory_callback: Optional[Callable] = None,
            camera_callback: Optional[Callable] = None,
            odom_topic: str = 'odom',
            tf_topic: str = '/tf',
            trajectory_topic: str = 'trajectory',
            camera_topic_sim: str = '/camera/image/raw',
            camera_topic_real: str = '/camera/img_raw'
        ):
            """
            Initialize Go2 ROS2 node

            Args:
                namespace: Robot namespace (e.g., 'robot0')
                odom_callback: Callback function for odometry data
            """
            super().__init__('go2_web_interface_bridge')

            self.namespace = namespace
            self.odom_callback = odom_callback
            self.tf_callback = tf_callback
            self.trajectory_callback = trajectory_callback
            self.camera_callback = camera_callback
            self.camera_topic_sim = camera_topic_sim
            self.camera_topic_real = camera_topic_real

            # QoS profile
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10
            )

            # QoS for higher-frequency streams
            fast_topic_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=50
            )

            # Topic names with namespace
            odom_topic_resolved = self._resolve_topic(odom_topic or 'odom')
            goal_topic = f'{namespace}/goal_pose' if namespace else '/goal_pose'
            goal_point_topic = self._resolve_topic('/goal_point')
            waypoint_topic = self._resolve_topic('/sim_way_point')
            real_waypoint_topic = self._resolve_topic('/real_way_point')
            cmd_vel_topic = f'{namespace}/cmd_vel_out' if namespace else 'cmd_vel_out'

            # Subscribers
            self.odom_sub = self.create_subscription(
                Odometry,
                odom_topic_resolved,
                self._on_odometry,
                qos
            )

            # Optional TF subscriber
            self.tf_sub = None
            tf_topic_resolved = self._resolve_topic(tf_topic)
            if self.tf_callback:
                self.tf_sub = self.create_subscription(
                    TFMessage,
                    tf_topic_resolved,
                    self._on_tf,
                    fast_topic_qos
                )

            # Optional trajectory subscriber
            trajectory_topic_resolved = self._resolve_topic(trajectory_topic)
            self.trajectory_sub = None
            if self.trajectory_callback:
                self.trajectory_sub = self.create_subscription(
                    PointCloud2,
                    trajectory_topic_resolved,
                    self._on_trajectory,
                    fast_topic_qos
                )

            # Optional camera subscribers (dual topics for simulation/real modes)
            camera_topic_sim_resolved = self._resolve_topic(camera_topic_sim)
            camera_topic_real_resolved = self._resolve_topic(camera_topic_real)
            self.camera_sub_sim = None
            self.camera_sub_real = None
            if self.camera_callback:
                if camera_topic_sim_resolved:
                    self.camera_sub_sim = self.create_subscription(
                        ROSImage,
                        camera_topic_sim_resolved,
                        self._on_camera_sim,
                        fast_topic_qos
                    )
                if camera_topic_real_resolved:
                    self.camera_sub_real = self.create_subscription(
                        ROSImage,
                        camera_topic_real_resolved,
                        self._on_camera_real,
                        fast_topic_qos
                    )

            # Publishers
            self.goal_pub = self.create_publisher(
                PoseStamped,
                goal_topic,
                qos
            )

            self.goal_point_pub = self.create_publisher(
                PointStamped,
                goal_point_topic,
                qos
            )

            self.waypoint_pub = self.create_publisher(
                PointStamped,
                waypoint_topic,
                qos
            )

            self.real_waypoint_pub = self.create_publisher(
                PointStamped,
                real_waypoint_topic,
                qos
            )

            self.cmd_vel_pub = self.create_publisher(
                Twist,
                cmd_vel_topic,
                qos
            )

            self.get_logger().info(f'Go2 web interface bridge initialized')
            self.get_logger().info(f'Subscribed to odom topic: {odom_topic_resolved}')
            if self.tf_sub:
                self.get_logger().info(f'Subscribed to TF topic: {tf_topic_resolved}')
            if self.trajectory_sub:
                self.get_logger().info(f'Subscribed to trajectory topic: {trajectory_topic_resolved}')
            if self.camera_sub_sim:
                self.get_logger().info(f'Subscribed to camera topic (simulation): {camera_topic_sim_resolved}')
            if self.camera_sub_real:
                self.get_logger().info(f'Subscribed to camera topic (real): {camera_topic_real_resolved}')
            self.get_logger().info(
                f'Publishing to: {goal_topic}, {goal_point_topic}, {waypoint_topic}, {cmd_vel_topic}'
            )

        def _resolve_topic(self, topic: str) -> str:
            """Resolve a potentially relative topic name with the node namespace"""
            if not topic:
                return topic

            if topic.startswith('/'):
                return topic

            base = topic.lstrip('/')
            if self.namespace:
                return f'{self.namespace}/{base}'
            return base

        def _on_odometry(self, msg):
            """Internal odometry callback"""
            if self.odom_callback:
                self.odom_callback(msg)

        def _on_tf(self, msg):
            """Internal TF callback"""
            if self.tf_callback:
                self.tf_callback(msg)

        def _on_trajectory(self, msg):
            """Internal trajectory callback"""
            if self.trajectory_callback:
                self.trajectory_callback(msg)

        def _on_camera_sim(self, msg):
            """Internal camera callback for simulation topic"""
            if self.camera_callback:
                self.camera_callback(msg, self.camera_topic_sim)

        def _on_camera_real(self, msg):
            """Internal camera callback for real topic"""
            if self.camera_callback:
                self.camera_callback(msg, self.camera_topic_real)

        def publish_goal(self, x: float, y: float, frame_id: str = 'map', mode: str = 'none', robot_pos: dict = None):
            """Publish navigation goal based on current mode"""
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = frame_id

            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0

            # Orientation (facing forward)
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0

            point_goal = PointStamped()
            point_goal.header = goal.header
            point_goal.point.x = x
            point_goal.point.y = y
            point_goal.point.z = 0.0

            # Publish based on mode
            if mode == 'simulation':
                # Simulation mode: only publish to /sim_way_point
                self.waypoint_pub.publish(point_goal)
                self.get_logger().info(f'[SIMULATION] Published to /sim_way_point: ({x:.2f}, {y:.2f})')
            elif mode == 'real':
                # Calculate theta from robot to goal
                import math
                if robot_pos:
                    dx = x - robot_pos.get('x', 0.0)
                    dy = y - robot_pos.get('y', 0.0)
                    theta = math.atan2(dy, dx)
                else:
                    theta = 0.0

                # Real mode: only publish to /real_way_point with theta
                real_waypoint = PointStamped()
                real_waypoint.header.stamp = self.get_clock().now().to_msg()
                real_waypoint.header.frame_id = frame_id
                real_waypoint.point.x = x
                real_waypoint.point.y = y
                real_waypoint.point.z = -theta  # Store inverted theta in z field (negative = right, positive = left)

                self.real_waypoint_pub.publish(real_waypoint)
                self.get_logger().info(f'[REAL] Published to /real_way_point: ({x:.2f}, {y:.2f}, theta={-theta:.2f} rad)')
            else:
                # Default/None mode: publish to /goal_pose and /goal_point
                self.goal_pub.publish(goal)
                self.goal_point_pub.publish(point_goal)
                self.get_logger().info(f'Published to /goal_pose and /goal_point: ({x:.2f}, {y:.2f})')

        def publish_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float):
            """Publish velocity command"""
            twist = Twist()
            twist.linear.x = linear_x
            twist.linear.y = linear_y
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = angular_z

            self.cmd_vel_pub.publish(twist)

else:

    class Go2Node:
        """Placeholder node used when ROS2 dependencies are unavailable"""

        def __init__(self, **kwargs):
            _ = kwargs  # Suppress unused warning
            raise RuntimeError('ROS2 is not available. Cannot create Go2Node instance.')


# ============================================================================
# Flask Web Server
# ============================================================================

# Get base directory and paths
import os
BASE_DIR = Path(__file__).resolve().parent
WEB_DIR = BASE_DIR / 'web'
APP_STATIC_FOLDER = str(WEB_DIR)

# Flask app setup
app = Flask(__name__, static_folder=APP_STATIC_FOLDER)
CORS(app)

# Configuration from environment variables
ENABLE_ROS2 = os.getenv('ENABLE_ROS2', 'false').lower() == 'true'
ROBOT_NAMESPACE = os.getenv('ROBOT_NAMESPACE', '')
MAP_FRAME = os.getenv('MAP_FRAME', 'map')
ROBOT_FRAME = os.getenv('ROBOT_FRAME', 'base')
TF_TOPIC = os.getenv('TF_TOPIC', '/tf')
TRAJECTORY_TOPIC = os.getenv('TRAJECTORY_TOPIC', '/trajectory')
TRAJECTORY_PRUNE_RADIUS = float(os.getenv('TRAJECTORY_PRUNE_RADIUS', '0.25'))
CAMERA_TOPIC_SIM = os.getenv('CAMERA_TOPIC_SIM', '/camera/image/raw')
CAMERA_TOPIC_REAL = os.getenv('CAMERA_TOPIC_REAL', '/camera/img_raw')
ODOM_TOPIC = os.getenv('ODOM_TOPIC', 'odom')

PROJECT_ROOT = BASE_DIR.parents[2] if len(BASE_DIR.parents) >= 3 else BASE_DIR
SAMPLE_TF_PATH = os.path.join(str(PROJECT_ROOT), 'tf.txt')
SAMPLE_TRAJECTORY_PATH = os.path.join(str(PROJECT_ROOT), 'trajectory.txt')

# Global ROS2 bridge instance
ros2_bridge: Optional[Go2ROS2Bridge] = None

# Map configuration
DEFAULT_MAP_BASE_PATH = str(BASE_DIR / 'maps')
MAP_BASE_PATH = os.path.abspath(os.getenv('MAP_BASE_PATH', DEFAULT_MAP_BASE_PATH))
CURRENT_MAP = 'cabinet_unity'

# Dummy robot position for testing
robot_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
goal_position = {'x': None, 'y': None}

# Sample data containers
SAMPLE_ROBOT_POSITION: Optional[Dict] = None
SAMPLE_TRAJECTORY: Dict = {'frame_id': MAP_FRAME, 'points': []}


def _load_sample_robot_pose(tf_path: str) -> Optional[Dict]:
    """Load a sample robot pose from a TF YAML file"""
    if not os.path.exists(tf_path):
        return None

    try:
        with open(tf_path, 'r') as handle:
            documents = yaml.safe_load_all(handle)
            for doc in documents:
                if not doc or 'transforms' not in doc:
                    continue
                for transform in doc.get('transforms', []):
                    translation = transform.get('transform', {}).get('translation', {})
                    rotation = transform.get('transform', {}).get('rotation', {})

                    x = float(translation.get('x', 0.0))
                    y = float(translation.get('y', 0.0))
                    theta = Go2ROS2Bridge._quaternion_to_yaw(
                        float(rotation.get('x', 0.0)),
                        float(rotation.get('y', 0.0)),
                        float(rotation.get('z', 0.0)),
                        float(rotation.get('w', 1.0))
                    )

                    return {'x': x, 'y': y, 'theta': theta}
    except Exception as exc:
        print(f"Failed to load sample TF data: {exc}")

    return None


def _load_sample_trajectory(trajectory_path: str) -> Dict:
    """Load a sample trajectory from a PointCloud2 YAML file"""
    if not os.path.exists(trajectory_path):
        return {'frame_id': MAP_FRAME, 'points': []}

    try:
        with open(trajectory_path, 'r') as handle:
            documents = list(yaml.safe_load_all(handle))

        if not documents:
            return {'frame_id': MAP_FRAME, 'points': []}

        msg = documents[0]
        width = int(msg.get('width', 0))
        height = int(msg.get('height', 0))
        point_step = int(msg.get('point_step', 0))
        fields = msg.get('fields', [])
        raw_data_list = msg.get('data', [])

        if isinstance(raw_data_list, (bytes, bytearray)):
            data = bytes(raw_data_list)
        else:
            byte_values = [value for value in raw_data_list if isinstance(value, int) and 0 <= value <= 255]
            data = bytes(byte_values)

        if width * height == 0 or point_step == 0 or not data:
            return {'frame_id': msg.get('frame_id', MAP_FRAME), 'points': []}

        x_offset = next((field.get('offset') for field in fields if field.get('name') == 'x'), None)
        y_offset = next((field.get('offset') for field in fields if field.get('name') == 'y'), None)

        if x_offset is None or y_offset is None:
            return {'frame_id': msg.get('frame_id', MAP_FRAME), 'points': []}

        little_endian = not bool(msg.get('is_bigendian', False))
        endian_prefix = '<' if little_endian else '>'

        total_points = width * height
        stride = max(1, total_points // 1000)

        points = []
        for index in range(0, total_points, stride):
            offset = index * point_step
            try:
                x = struct.unpack_from(f'{endian_prefix}f', data, offset + x_offset)[0]
                y = struct.unpack_from(f'{endian_prefix}f', data, offset + y_offset)[0]
            except struct.error:
                continue

            if math.isfinite(x) and math.isfinite(y):
                points.append({'x': float(x), 'y': float(y)})

        return {
            'frame_id': (msg.get('header', {}).get('frame_id') or msg.get('frame_id') or MAP_FRAME).lstrip('/'),
            'points': points
        }
    except Exception as exc:
        print(f"Failed to load sample trajectory data: {exc}")

    return {'frame_id': MAP_FRAME, 'points': []}


def _prune_points_in_place(points: list, robot_pose: Dict, radius: float) -> int:
    """Remove trajectory points that are within the radius of the robot pose"""
    if not points or not robot_pose or radius <= 0:
        return 0

    rx = float(robot_pose.get('x', 0.0))
    ry = float(robot_pose.get('y', 0.0))
    radius_sq = radius * radius

    remaining = []
    for point in points:
        px = float(point.get('x', 0.0))
        py = float(point.get('y', 0.0))
        if ((px - rx) * (px - rx) + (py - ry) * (py - ry)) > radius_sq:
            remaining.append(point)

    removed = len(points) - len(remaining)
    if removed:
        points[:] = remaining

    return removed


def _candidate_image_paths(yaml_dir: str, map_name: str, image_value: Optional[str]):
    """Yield plausible absolute paths for the map image."""
    candidates = []

    if image_value:
        if os.path.isabs(image_value):
            candidates.append(image_value)
        else:
            candidates.append(os.path.normpath(os.path.join(yaml_dir, image_value)))

        # Try keeping only the filename in the same folder
        candidates.append(os.path.join(yaml_dir, os.path.basename(image_value)))

        # Try locating assets within the configured map directory
        candidates.append(os.path.join(MAP_BASE_PATH, os.path.basename(image_value)))
        candidates.append(os.path.join(MAP_BASE_PATH, map_name, os.path.basename(image_value)))

    for extension in ('.pgm', '.png', '.jpg', '.jpeg'):
        candidates.append(os.path.join(yaml_dir, f'{map_name}{extension}'))

    seen = set()
    for path in candidates:
        if not path or path in seen:
            continue
        seen.add(path)
        yield path


def load_map_metadata(map_name: str) -> Dict:
    """Load map YAML metadata and resolve the image location."""
    yaml_path = os.path.join(MAP_BASE_PATH, f'{map_name}.yaml')
    with open(yaml_path, 'r') as handle:
        metadata = yaml.safe_load(handle) or {}

    yaml_dir = os.path.dirname(yaml_path)
    image_value = metadata.get('image')

    resolved_path = None
    for candidate in _candidate_image_paths(yaml_dir, map_name, image_value):
        if os.path.exists(candidate):
            resolved_path = os.path.abspath(candidate)
            break

    if not resolved_path:
        raise FileNotFoundError(
            f"Map image not found for '{map_name}'. Checked paths derived from '{yaml_path}'."
        )

    metadata['image_path'] = resolved_path
    metadata['yaml_path'] = yaml_path
    return metadata


def load_map_image(metadata: Dict) -> Dict:
    """Load map raster (pgm/png/jpg) and convert to PNG base64."""
    image_path = metadata.get('image_path')
    if not image_path or not os.path.exists(image_path):
        raise FileNotFoundError(f"Map image not found at {image_path}")

    img = PILImage.open(image_path)
    img_rgb = img.convert('RGB')

    buffer = io.BytesIO()
    img_rgb.save(buffer, format='PNG')
    img_base64 = base64.b64encode(buffer.getvalue()).decode()

    return {
        'data': f'data:image/png;base64,{img_base64}',
        'width': img.width,
        'height': img.height
    }


# ============================================================================
# Flask Routes
# ============================================================================

@app.route('/')
def index():
    """Serve the main HTML page"""
    return send_from_directory(APP_STATIC_FOLDER, 'index.html')


@app.route('/style.css')
def serve_css():
    """Serve the CSS file"""
    return send_from_directory(APP_STATIC_FOLDER, 'style.css')


@app.route('/app.js')
def serve_js():
    """Serve the JavaScript file"""
    return send_from_directory(APP_STATIC_FOLDER, 'app.js')


@app.route('/ChatGPT_Image_Oct_3__2025__11_46_27_PM-removebg-preview.png')
def serve_robot_image():
    """Serve the robot PNG image"""
    return send_from_directory(APP_STATIC_FOLDER, 'ChatGPT_Image_Oct_3__2025__11_46_27_PM-removebg-preview.png')


@app.route('/api/map')
def get_map():
    """Get map data including image and metadata"""
    try:
        metadata = load_map_metadata(CURRENT_MAP)
        image_data = load_map_image(metadata)

        return jsonify({
            'success': True,
            'map': {
                'name': CURRENT_MAP,
                'image': image_data['data'],
                'width': image_data['width'],
                'height': image_data['height'],
                'resolution': metadata.get('resolution', 0.05),
                'origin': metadata.get('origin', [0.0, 0.0, 0.0]),
                'occupied_thresh': metadata.get('occupied_thresh', 0.65),
                'free_thresh': metadata.get('free_thresh', 0.196)
            }
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/position', methods=['GET', 'POST'])
def handle_position():
    """Get or update robot position"""
    global robot_position

    if request.method == 'GET':
        # Use ROS2 bridge if enabled, otherwise use dummy position
        if ENABLE_ROS2 and ros2_bridge and ros2_bridge.enabled:
            position = ros2_bridge.get_robot_position()
            mode = 'ros2'
        else:
            # Use raw position (transformation will be applied in frontend)
            position = SAMPLE_ROBOT_POSITION or robot_position
            mode = 'sample' if SAMPLE_ROBOT_POSITION else 'dummy'

        return jsonify({
            'success': True,
            'position': position,
            'mode': mode,
            'connected': ros2_bridge.is_connected() if (ENABLE_ROS2 and ros2_bridge) else False
        })

    elif request.method == 'POST':
        # Only allow manual position updates in dummy mode
        if ENABLE_ROS2 and ros2_bridge and ros2_bridge.enabled:
            return jsonify({
                'success': False,
                'error': 'Cannot manually set position in ROS2 mode'
            }), 400

        data = request.json
        if 'x' in data:
            robot_position['x'] = float(data['x'])
        if 'y' in data:
            robot_position['y'] = float(data['y'])
        if 'theta' in data:
            robot_position['theta'] = float(data['theta'])

        return jsonify({
            'success': True,
            'position': robot_position
        })


@app.route('/api/trajectory', methods=['GET'])
def get_trajectory():
    """Return the latest trajectory points"""
    if ENABLE_ROS2 and ros2_bridge and ros2_bridge.enabled:
        trajectory = ros2_bridge.get_trajectory(prune_radius=TRAJECTORY_PRUNE_RADIUS)
        mode = 'ros2'
    else:
        _prune_points_in_place(SAMPLE_TRAJECTORY['points'], robot_position, TRAJECTORY_PRUNE_RADIUS)
        trajectory = {
            'frame_id': SAMPLE_TRAJECTORY.get('frame_id', MAP_FRAME),
            'points': [
                {'x': float(point.get('x', 0.0)), 'y': float(point.get('y', 0.0))}
                for point in SAMPLE_TRAJECTORY.get('points', [])
            ]
        }
        mode = 'sample' if SAMPLE_TRAJECTORY.get('points') else 'dummy'

    return jsonify({
        'success': True,
        'trajectory': trajectory,
        'mode': mode
    })


@app.route('/api/camera', methods=['GET'])
def get_camera_frame():
    """Return the most recent camera frame"""
    if not (ENABLE_ROS2 and ros2_bridge and ros2_bridge.enabled):
        return jsonify({
            'success': False,
            'error': 'ROS2 mode disabled or unavailable'
        }), 503

    frame = ros2_bridge.get_latest_camera_image(max_age=2.0)
    if not frame:
        return jsonify({
            'success': False,
            'error': 'No camera frame available'
        }), 503

    return jsonify({
        'success': True,
        'image': f"data:image/{frame['format']};base64,{frame['data']}",
        'timestamp': frame['timestamp'],
        'encoding': frame['encoding'],
        'width': frame['width'],
        'height': frame['height']
    })


@app.route('/api/goal', methods=['GET', 'POST'])
def handle_goal():
    """Get or set goal position"""
    global goal_position

    if request.method == 'GET':
        return jsonify({
            'success': True,
            'goal': goal_position
        })

    elif request.method == 'POST':
        data = request.json
        x = float(data['x']) if data.get('x') is not None else None
        y = float(data['y']) if data.get('y') is not None else None

        # Update local goal storage
        goal_position['x'] = x
        goal_position['y'] = y

        # Send to ROS2 if enabled and goal is valid
        if ENABLE_ROS2 and ros2_bridge and ros2_bridge.enabled and x is not None and y is not None:
            success = ros2_bridge.send_navigation_goal(x, y, frame_id='map')
            current_mode = ros2_bridge.get_mode()
            return jsonify({
                'success': success,
                'goal': goal_position,
                'mode': 'ros2',
                'operating_mode': current_mode,
                'message': 'Goal sent to robot' if success else 'Failed to send goal to robot'
            })

        return jsonify({
            'success': True,
            'goal': goal_position,
            'mode': 'dummy'
        })


@app.route('/api/maps')
def list_maps():
    """List available maps"""
    try:
        maps = []
        for file in os.listdir(MAP_BASE_PATH):
            if file.endswith('.yaml'):
                map_name = file.replace('.yaml', '')
                maps.append(map_name)

        return jsonify({
            'success': True,
            'maps': maps,
            'current': CURRENT_MAP
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/maps/<map_name>', methods=['POST'])
def switch_map(map_name: str):
    """Switch to a different map"""
    global CURRENT_MAP

    try:
        # Verify map exists
        yaml_path = os.path.join(MAP_BASE_PATH, f'{map_name}.yaml')
        if not os.path.exists(yaml_path):
            return jsonify({
                'success': False,
                'error': 'Map not found'
            }), 404

        CURRENT_MAP = map_name

        # Update map offset in ROS2 bridge if enabled
        if ros2_bridge is not None:
            ros2_bridge.update_map_offset(map_name)

        return jsonify({
            'success': True,
            'current': CURRENT_MAP
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/mode', methods=['GET'])
def get_mode():
    """Get current operating mode"""
    mode = ros2_bridge.get_mode() if ros2_bridge else 'none'
    return jsonify({
        'success': True,
        'mode': mode
    })


@app.route('/api/set_mode/<mode>', methods=['POST'])
def set_mode(mode: str):
    """Set operating mode and launch appropriate script"""
    import subprocess

    if mode not in ['simulation', 'real']:
        return jsonify({
            'success': False,
            'error': 'Invalid mode. Must be "simulation" or "real"'
        }), 400

    try:
        # Determine the script/command to run
        if mode == 'simulation':
            # Path to simulation script (in user's home directory)
            # Launch in new terminal with cd to autonomy_stack_go2
            command = [
                'gnome-terminal',
                '--',
                'bash',
                '-c',
                'cd ~/autonomy_stack_go2 && ./system_simulation.sh; exec bash'
            ]

        else:  # mode == 'real'
            pass

        # Execute the command to open terminal
        subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # Update mode in bridge if available
        if ros2_bridge:
            ros2_bridge.set_mode(mode)

        return jsonify({
            'success': True,
            'mode': mode,
            'message': f'{mode.capitalize()} mode activated. Check terminal window.'
        })

    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/object_ids', methods=['GET'])
def get_object_ids():
    """Load object IDs from YAML file"""
    object_ids_path = Path(MAP_BASE_PATH) / 'object_ids.yaml'

    try:
        if not object_ids_path.exists():
            # Return empty list if file doesn't exist
            return jsonify({
                'success': True,
                'object_ids': []
            })

        with open(object_ids_path, 'r') as f:
            data = yaml.safe_load(f)

        object_ids = data.get('object_ids', []) if data else []

        return jsonify({
            'success': True,
            'object_ids': object_ids
        })

    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/object_ids', methods=['POST'])
def save_object_ids():
    """Save object IDs to YAML file"""
    object_ids_path = Path(MAP_BASE_PATH) / 'object_ids.yaml'

    try:
        data = request.get_json()
        object_ids = data.get('object_ids', [])

        # Validate that object_ids is a list
        if not isinstance(object_ids, list):
            return jsonify({
                'success': False,
                'error': 'object_ids must be a list'
            }), 400

        # Save to YAML file
        yaml_data = {
            'object_ids': object_ids
        }

        with open(object_ids_path, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)

        return jsonify({
            'success': True,
            'message': f'Saved {len(object_ids)} object IDs to {object_ids_path}'
        })

    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


# ============================================================================
# Main Entry Point
# ============================================================================

def main():
    """Main entry point for the Go2 web interface"""
    global ros2_bridge, SAMPLE_ROBOT_POSITION, SAMPLE_TRAJECTORY, robot_position

    print("=" * 60)
    print("Initializing Go2 Web Interface")
    print("=" * 60)
    print(f"Map base path: {os.path.abspath(MAP_BASE_PATH)}")
    print(f"Current map: {CURRENT_MAP}")

    # Load sample data
    SAMPLE_ROBOT_POSITION = _load_sample_robot_pose(SAMPLE_TF_PATH)
    SAMPLE_TRAJECTORY = _load_sample_trajectory(SAMPLE_TRAJECTORY_PATH)

    if SAMPLE_ROBOT_POSITION:
        robot_position = SAMPLE_ROBOT_POSITION

    # Initialize ROS2 bridge if enabled
    if ENABLE_ROS2:
        print(f"\nROS2 mode enabled")
        print(f"  Namespace: {ROBOT_NAMESPACE or '(default)'}")
        print(f"  Odometry topic: {ODOM_TOPIC}")
        print(f"  TF topic: {TF_TOPIC}")
        print(f"  Trajectory topic: {TRAJECTORY_TOPIC}")
        print(f"  Camera topic (simulation): {CAMERA_TOPIC_SIM}")
        print(f"  Camera topic (real): {CAMERA_TOPIC_REAL}")
        print(f"Initializing ROS2 bridge...")

        ros2_bridge = Go2ROS2Bridge(
            enable_ros2=ENABLE_ROS2,
            robot_namespace=ROBOT_NAMESPACE,
            map_frame=MAP_FRAME,
            robot_frame=ROBOT_FRAME,
            tf_topic=TF_TOPIC,
            trajectory_topic=TRAJECTORY_TOPIC,
            camera_topic_sim=CAMERA_TOPIC_SIM,
            camera_topic_real=CAMERA_TOPIC_REAL,
            odom_topic=ODOM_TOPIC,
            map_name=CURRENT_MAP
        )

        if ros2_bridge.enabled:
            print("✓ ROS2 bridge initialized successfully")
        else:
            print("✗ ROS2 bridge initialization failed")
    else:
        print("\nRunning in dummy mode (ROS2 disabled)")

    print("=" * 60)
    print(f"Starting Flask server at http://0.0.0.0:8080")
    print("=" * 60)

    # Start the Flask server
    try:
        app.run(debug=True, host='0.0.0.0', port=8080)
    finally:
        # Cleanup
        if ros2_bridge is not None:
            print("\nShutting down ROS2 bridge...")
            ros2_bridge.shutdown()
            print("Cleanup complete")


if __name__ == '__main__':
    main()
