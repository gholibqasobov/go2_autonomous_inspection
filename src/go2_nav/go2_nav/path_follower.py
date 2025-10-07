#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from nav_msgs.msg import Path
import tf2_ros
import math
import numpy as np

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        
        # Publisher for requesting new path planning
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Control parameters - Optimized for better path following
        self.k_linear = 2.0          # Linear velocity gain (reduced for stability) 0.8
        self.k_angular = 6.0         # Angular velocity gain (increased for better steering) 3.0
        self.max_linear_vel = 2.0    # Maximum linear velocity (reduced for control)
        self.max_angular_vel = 3.0   # Maximum angular velocity (slightly increased) 1.2
        self.goal_tolerance = 0.18   # Distance tolerance to waypoint (tighter) 12
        self.angle_tolerance = 0.15   # Angle tolerance (tighter for precision)
        self.lookahead_distance = 0.4 # Lookahead distance (reduced for tighter following)
        
        # Path corridor following parameters
        self.path_corridor_width = 0.35  # Allow deviation up to 0.25m from path
        self.waypoint_skip_distance = 0.2  # Skip waypoints within this distance
        self.adaptive_lookahead = True   # Use adaptive lookahead based on speed
        self.min_lookahead = 0.2        # Minimum lookahead distance
        self.max_lookahead = 0.8        # Maximum lookahead distance
        
        # Rotation-first behavior parameters
        self.rotation_threshold = math.pi / 3  # 60 degrees - rotate first if angle > this
        self.rotation_tolerance = 0.1          # Stop rotating when within this angle
        self.rotation_speed = 0.8              # Angular velocity during rotation-only phase

        # State variables
        self.path = []
        self.current_pose = None
        self.current_index = 0
        self.reached_goal = False
        self.path_active = False
        
        # Dynamic replanning variables - Adjusted for corridor following
        self.final_goal = None  # Store the ultimate goal
        self.last_replan_pose = None  # Last position where we replanned
        self.replan_distance_threshold = 0.4  # Replan if robot moves this far from last replan
        self.replan_deviation_threshold = self.path_corridor_width * 1.5  # Replan if robot goes outside corridor buffer
        self.min_replan_interval = 1.0  # Minimum time between replans (seconds)
        self.last_replan_time = 0.0
        self.enable_dynamic_replanning = True  # Enable/disable dynamic replanning

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz (increased from 10 Hz)

        self.get_logger().info("Path follower node started")

    def path_callback(self, msg):
        """Callback when new path is received"""
        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty path")
            return
            
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.current_index = 0
        self.reached_goal = False
        self.path_active = True
        
        # Store the final goal for dynamic replanning
        if len(self.path) > 0:
            self.final_goal = self.path[-1]
            
        # Update last replan position
        if self.current_pose:
            self.last_replan_pose = (self.current_pose[0], self.current_pose[1])
            
        self.get_logger().info(f"Received new path with {len(self.path)} waypoints")
        if self.final_goal:
            self.get_logger().info(f"Final goal: ({self.final_goal[0]:.3f}, {self.final_goal[1]:.3f})")

    def get_robot_pose(self):
        """Get current robot pose from TF"""
        try:
            # Try to get the latest available transform with timeout
            trans = self.tf_buffer.lookup_transform(
                'map', 'vehicle', rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=0.1))
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            # Convert quaternion to yaw
            quat = trans.transform.rotation
            yaw = math.atan2(
                2.0 * (quat.w * quat.z + quat.x * quat.y),
                1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
            )
            
            # Store last valid pose
            self.last_valid_pose = (x, y, yaw)
            return x, y, yaw
            
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            # Try to use last known pose if available
            if hasattr(self, 'last_valid_pose'):
                self.get_logger().warn(f"Using last known pose: {self.last_valid_pose}")
                return self.last_valid_pose
            return None, None, None

    def control_loop(self):
        """Main control loop"""
        if not self.path_active or self.reached_goal:
            return

        # Get current robot pose
        pose_result = self.get_robot_pose()
        if pose_result[0] is None:  # Check if TF lookup failed
            self.get_logger().warn("Cannot get robot pose, skipping control loop")
            return

        robot_x, robot_y, robot_yaw = pose_result
        self.current_pose = pose_result

        # Check for dynamic replanning
        if self.should_replan(robot_x, robot_y):
            self.request_replan()
            return  # Skip this control cycle to wait for new path
        
        # Add a simple test: replan every 5 seconds for testing
        current_time = self.get_clock().now().nanoseconds / 1e9
        if (current_time - self.last_replan_time > 5.0 and 
            self.final_goal and 
            len(self.path) > 0):
            self.get_logger().info("Testing: Periodic replanning every 5 seconds")
            self.request_replan()
            return

        # Check if we've reached the end of the path
        if self.current_index >= len(self.path):
            self.stop_robot()
            self.reached_goal = True
            self.path_active = False
            self.get_logger().info("Goal reached!")
            return

        # Debug: Print current robot pose and target waypoint with corridor info
        if self.current_index < len(self.path):
            target_x, target_y = self.path[self.current_index]
            distance_to_target = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)
            path_deviation = self.distance_to_path(robot_x, robot_y)
            corridor_status = "IN CORRIDOR" if path_deviation < self.path_corridor_width else "OFF TRACK"
            self.get_logger().info(f"Robot: ({robot_x:.3f}, {robot_y:.3f}), Target: ({target_x:.3f}, {target_y:.3f}), "
                                 f"Distance: {distance_to_target:.3f}m, Path deviation: {path_deviation:.3f}m ({corridor_status})")

        # Find the target waypoint (with corridor-aware lookahead)
        target_x, target_y = self.find_lookahead_point(robot_x, robot_y)

        # Check distance from path for safety
        path_deviation = self.distance_to_path(robot_x, robot_y)
        
        # Compute control commands
        cmd = self.compute_control(robot_x, robot_y, robot_yaw, target_x, target_y)
        
        # Apply safety speed reduction if far from path
        if path_deviation > self.replan_deviation_threshold:
            safety_factor = max(0.3, 1.0 - (path_deviation - self.replan_deviation_threshold) / 0.2)
            cmd.twist.linear.x *= safety_factor
            self.get_logger().warn(f"Far from path (dev: {path_deviation:.2f}m), reducing speed by {1-safety_factor:.1%}")
        
        # Publish command
        self.cmd_pub.publish(cmd)

    def find_lookahead_point(self, robot_x, robot_y):
        """Find lookahead point with corridor following - skip waypoints if robot is within corridor"""
        if not self.path:
            return None, None
            
        # Check if robot is within corridor of the path
        path_deviation = self.distance_to_path(robot_x, robot_y)
        
        # Adaptive lookahead based on current speed and deviation
        current_lookahead = self.lookahead_distance
        if self.adaptive_lookahead:
            # Increase lookahead if we're on track, decrease if we're off track
            if path_deviation < self.path_corridor_width * 0.5:
                current_lookahead = min(self.max_lookahead, self.lookahead_distance * 1.3)
            elif path_deviation > self.path_corridor_width:
                current_lookahead = max(self.min_lookahead, self.lookahead_distance * 0.7)
        
        # Skip waypoints that are close to robot (within corridor)
        while self.current_index < len(self.path):
            waypoint_x, waypoint_y = self.path[self.current_index]
            distance_to_waypoint = math.sqrt((waypoint_x - robot_x)**2 + (waypoint_y - robot_y)**2)
            
            # Check if we should skip this waypoint
            should_skip = False
            
            # Skip if waypoint is very close (within skip distance)
            if distance_to_waypoint < self.waypoint_skip_distance:
                should_skip = True
                
            # Also skip if robot is roughly aligned with path direction and within corridor
            elif (distance_to_waypoint < self.goal_tolerance * 2 and 
                  path_deviation < self.path_corridor_width):
                should_skip = True
            
            if should_skip:
                self.get_logger().info(f"Skipping waypoint {self.current_index} (distance: {distance_to_waypoint:.3f}m, corridor deviation: {path_deviation:.3f}m)")
                self.current_index += 1
            else:
                break
        
        # If we've reached the end, return the last waypoint
        if self.current_index >= len(self.path):
            return self.path[-1]
        
        # Find the best lookahead point
        best_point = self.path[self.current_index]  # Default to current waypoint
        
        # Look ahead for a point at the desired lookahead distance
        for i in range(self.current_index, len(self.path)):
            waypoint_x, waypoint_y = self.path[i]
            distance = math.sqrt((waypoint_x - robot_x)**2 + (waypoint_y - robot_y)**2)
            
            # If this point is at or beyond current lookahead distance, use it
            if distance >= current_lookahead:
                best_point = (waypoint_x, waypoint_y)
                break
            else:
                # Keep updating best_point as we search
                best_point = (waypoint_x, waypoint_y)
        
        return best_point

    def compute_control(self, robot_x, robot_y, robot_yaw, target_x, target_y):
        """Compute control commands with rotation-first behavior for large angles"""
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = "vehicle"
        
        cmd = cmd_msg.twist  # Get the Twist part of TwistStamped
        
        # Compute distance and angle to target
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_target = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_target - robot_yaw)

        # ROTATION-FIRST BEHAVIOR: For large angles, rotate first, then move
        if abs(angle_error) > self.rotation_threshold:
            # Large angle error (>60°) - ROTATE IN PLACE FIRST
            self.get_logger().info(f"Large angle error ({angle_error*180/math.pi:.1f}°) - Rotating in place")
            
            # Pure rotation - NO forward movement
            cmd.linear.x = 0.0
            
            # Angular velocity proportional to error, but with minimum speed
            if abs(angle_error) > self.rotation_tolerance:
                angular_speed = self.k_angular * angle_error
                # Ensure minimum rotation speed for large errors
                min_rotation = self.rotation_speed * (1.0 if abs(angle_error) > math.pi/2 else 0.6)
                if angular_speed > 0:
                    angular_speed = max(min_rotation, angular_speed)
                else:
                    angular_speed = min(-min_rotation, angular_speed)
                    
                cmd.angular.z = max(-self.max_angular_vel, min(angular_speed, self.max_angular_vel))
            else:
                cmd.angular.z = 0.0
                
        elif abs(angle_error) > self.angle_tolerance:
            # Medium angle error - combine rotation and forward motion
            self.get_logger().debug(f"Medium angle error ({angle_error*180/math.pi:.1f}°) - Combined movement")
            
            # Reduced forward speed while turning
            linear_speed = self.k_linear * distance * (1.0 - abs(angle_error) / self.rotation_threshold)
            linear_speed = max(0.1, min(linear_speed, self.max_linear_vel * 0.6))  # Max 60% speed while turning
            
            angular_speed = self.k_angular * angle_error
            cmd.linear.x = linear_speed
            cmd.angular.z = max(-self.max_angular_vel, min(angular_speed, self.max_angular_vel))
            
        else:
            # Small angle error - full forward motion with gentle steering
            self.get_logger().debug(f"Small angle error ({angle_error*180/math.pi:.1f}°) - Forward motion")
            
            linear_speed = self.k_linear * distance
            linear_speed = min(linear_speed, self.max_linear_vel)
            
            # Gentle angular correction
            angular_speed = self.k_angular * angle_error * 0.5
            
            cmd.linear.x = max(0.0, linear_speed)
            cmd.angular.z = max(-self.max_angular_vel * 0.5, min(angular_speed, self.max_angular_vel * 0.5))

        # Debug output for tuning
        if self.get_clock().now().nanoseconds % 1000000000 < 100000000:  # Print every ~1 second
            self.get_logger().info(f"Control: distance={distance:.2f}, angle_err={angle_error*180/math.pi:.1f}°, "
                                 f"lin_vel={cmd.linear.x:.2f}, ang_vel={cmd.angular.z:.2f}")

        return cmd_msg

    def stop_robot(self):
        """Stop the robot"""
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = "vehicle"
        # Twist is already initialized to zero
        self.cmd_pub.publish(cmd_msg)
        self.get_logger().info("Robot stopped")

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def should_replan(self, robot_x, robot_y):
        """Check if we should trigger dynamic replanning"""
        if not self.enable_dynamic_replanning or not self.final_goal:
            self.get_logger().debug("Dynamic replanning disabled or no final goal")
            return False
            
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Don't replan too frequently
        if current_time - self.last_replan_time < self.min_replan_interval:
            self.get_logger().debug(f"Too soon to replan: {current_time - self.last_replan_time:.1f}s < {self.min_replan_interval}s")
            return False
            
        # Check if robot has moved significantly since last replan
        if self.last_replan_pose:
            distance_since_replan = math.sqrt(
                (robot_x - self.last_replan_pose[0])**2 + 
                (robot_y - self.last_replan_pose[1])**2
            )
            
            self.get_logger().debug(f"Distance since last replan: {distance_since_replan:.3f}m (threshold: {self.replan_distance_threshold}m)")
            
            if distance_since_replan > self.replan_distance_threshold:
                self.get_logger().info(f"Replanning due to distance: {distance_since_replan:.3f}m > {self.replan_distance_threshold}m")
                return True
        
        # Check if robot has deviated significantly from current path
        if len(self.path) > 1 and self.current_index < len(self.path) - 1:
            min_distance_to_path = self.distance_to_path(robot_x, robot_y)
            self.get_logger().debug(f"Distance to path: {min_distance_to_path:.3f}m (threshold: {self.replan_deviation_threshold}m)")
            
            if min_distance_to_path > self.replan_deviation_threshold:
                self.get_logger().info(f"Replanning due to path deviation: {min_distance_to_path:.3f}m > {self.replan_deviation_threshold}m")
                return True
        else:
            self.get_logger().debug("Not enough path points for deviation check")
                
        return False

    def distance_to_path(self, robot_x, robot_y):
        """Calculate minimum distance from robot to the remaining path"""
        if not self.path or self.current_index >= len(self.path):
            return 0.0
            
        min_distance = float('inf')
        
        # Check distance to remaining waypoints
        for i in range(self.current_index, len(self.path)):
            waypoint_x, waypoint_y = self.path[i]
            distance = math.sqrt((waypoint_x - robot_x)**2 + (waypoint_y - robot_y)**2)
            min_distance = min(min_distance, distance)
            
        # Check distance to line segments between consecutive waypoints
        for i in range(self.current_index, len(self.path) - 1):
            p1_x, p1_y = self.path[i]
            p2_x, p2_y = self.path[i + 1]
            distance = self.point_to_line_distance(robot_x, robot_y, p1_x, p1_y, p2_x, p2_y)
            min_distance = min(min_distance, distance)
            
        return min_distance

    def point_to_line_distance(self, px, py, x1, y1, x2, y2):
        """Calculate distance from point to line segment"""
        # Vector from point 1 to point 2
        dx = x2 - x1
        dy = y2 - y1
        
        if dx == 0 and dy == 0:
            # Line segment is a point
            return math.sqrt((px - x1)**2 + (py - y1)**2)
        
        # Parameter t for closest point on line
        t = ((px - x1) * dx + (py - y1) * dy) / (dx**2 + dy**2)
        t = max(0.0, min(1.0, t))  # Clamp to line segment
        
        # Closest point on line segment
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        # Distance to closest point
        return math.sqrt((px - closest_x)**2 + (py - closest_y)**2)

    def request_replan(self):
        """Request a new path plan by publishing the goal again"""
        if not self.final_goal:
            return
            
        # Create goal message
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = self.final_goal[0]
        goal_msg.pose.position.y = self.final_goal[1]
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0  # No specific orientation
        
        # Publish goal to trigger replanning
        self.goal_pub.publish(goal_msg)
        
        # Update replan tracking
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.last_replan_time = current_time
        if self.current_pose:
            self.last_replan_pose = (self.current_pose[0], self.current_pose[1])
            
        self.get_logger().info(f"Requested path replanning to goal: ({self.final_goal[0]:.3f}, {self.final_goal[1]:.3f})")

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Path follower shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
