#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import numpy as np
if not hasattr(np, 'float'):
    np.float = float
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Point
import heapq
import tf_transformations
import tf2_ros



class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')

        # QoS profile for map subscription (compatible with map servers)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Publisher
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Variables
        self.map = None
        self.map_info = None
        self.goal_pose = None
        self.inflated_map = None
        
        # Safety parameters
        self.robot_radius = 0.3  # Robot radius in meters (adjust as needed)
        self.safety_margin = 0.2  # Additional safety margin in meters

        # Timer to check map reception
        self.map_check_timer = self.create_timer(2.0, self.check_map_status)

        self.get_logger().info("A* Planner node started")

    # ----------------- Callbacks -----------------
    def check_map_status(self):
        """Periodically check if we have received a map"""
        if self.map is None:
            self.get_logger().info("Waiting for map data on /map topic...")
        else:
            self.get_logger().info(f"Map available: {self.map_info.width}x{self.map_info.height}, resolution: {self.map_info.resolution:.3f}m")
            # Cancel the timer once we have the map
            self.map_check_timer.cancel()

    def map_callback(self, msg):
        self.map = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        
        # Debug: Check map values to understand obstacle encoding
        unique_values = np.unique(self.map)
        self.get_logger().info(f"Map received: {msg.info.width}x{msg.info.height}, resolution: {msg.info.resolution:.3f}m")
        self.get_logger().info(f"Map unique values: {unique_values}")
        
        # Count different cell types
        free_cells = np.sum(self.map == 0)
        unknown_cells = np.sum(self.map == -1)
        obstacle_cells = np.sum(self.map == 100)
        self.get_logger().info(f"Free cells: {free_cells}, Unknown: {unknown_cells}, Obstacles: {obstacle_cells}")
        
        # Create inflated map for safer planning
        self.create_inflated_map()
    
    def create_inflated_map(self):
        """Create an inflated obstacle map for safer path planning"""
        if self.map is None:
            return
            
        resolution = self.map_info.resolution
        total_radius = self.robot_radius + self.safety_margin
        inflation_cells = int(np.ceil(total_radius / resolution))
        
        self.get_logger().info(f"Inflating obstacles by {inflation_cells} cells ({total_radius:.2f}m)")
        
        # Create binary obstacle map
        obstacles = (self.map > 50) | (self.map < 0)
        
        # Create inflated map with cost values
        self.inflated_map = np.zeros_like(self.map, dtype=np.int16)
        height, width = self.map.shape
        
        # For each cell, find distance to nearest obstacle
        for y in range(height):
            for x in range(width):
                if obstacles[y, x]:
                    # This is an obstacle
                    self.inflated_map[y, x] = 255  # High cost (impassable)
                else:
                    # Find minimum distance to any obstacle
                    min_dist = float('inf')
                    for dy in range(-inflation_cells, inflation_cells + 1):
                        for dx in range(-inflation_cells, inflation_cells + 1):
                            ny, nx = y + dy, x + dx
                            if 0 <= ny < height and 0 <= nx < width:
                                if obstacles[ny, nx]:
                                    dist = np.sqrt(dx*dx + dy*dy)
                                    min_dist = min(min_dist, dist)
                    
                    if min_dist <= inflation_cells:
                        # Apply cost based on distance to obstacle
                        # Closer to obstacle = higher cost
                        cost_factor = max(0, (inflation_cells - min_dist) / inflation_cells)
                        self.inflated_map[y, x] = int(50 * cost_factor)  # Cost 0-50
                    else:
                        self.inflated_map[y, x] = 0  # Free space
        
        inflated_obstacles = np.sum(self.inflated_map == 255)
        high_cost_cells = np.sum((self.inflated_map > 0) & (self.inflated_map < 255))
        self.get_logger().info(f"Inflation complete. Obstacles: {inflated_obstacles}, High-cost cells: {high_cost_cells}")

    def goal_callback(self, msg):
        if self.map is None:
            self.get_logger().warn("No map yet, cannot plan.")
            return

        self.goal_pose = msg.pose
        self.get_logger().info("Goal received, computing path...")
        self.plan_path()

    # ----------------- Helpers -----------------
    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'vehicle', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return x, y
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None, None

    def world_to_map(self, x, y):
        """Convert world coordinates (meters) to map grid indices"""
        origin = self.map_info.origin.position
        resolution = self.map_info.resolution
        orientation = self.map_info.origin.orientation
        
        # Apply inverse rotation to transform from world to map frame
        # The map has a rotation, so we need to account for it
        
        # Translate to origin
        dx = x - origin.x
        dy = y - origin.y
        
        # Apply inverse rotation (transpose of rotation matrix)
        # For quaternion (0, 0, -0.707, 0.707) which is -90° around Z:
        # Inverse rotation is +90° around Z
        cos_theta = orientation.w * orientation.w - orientation.z * orientation.z
        sin_theta = -2.0 * orientation.w * orientation.z
        
        # Rotate the point
        rotated_x = cos_theta * dx - sin_theta * dy
        rotated_y = sin_theta * dx + cos_theta * dy
        
        # Convert to map coordinates
        mx = int(rotated_x / resolution)
        my = int(rotated_y / resolution)
        
        return mx, my

    def map_to_world(self, mx, my):
        """Convert map grid indices to world coordinates"""
        origin = self.map_info.origin.position
        resolution = self.map_info.resolution
        orientation = self.map_info.origin.orientation
        
        # Convert to local coordinates
        local_x = mx * resolution
        local_y = my * resolution
        
        # Apply rotation (forward rotation matrix)
        # For quaternion (0, 0, -0.707, 0.707) which is -90° around Z:
        cos_theta = orientation.w * orientation.w - orientation.z * orientation.z
        sin_theta = 2.0 * orientation.w * orientation.z
        
        # Rotate the point
        rotated_x = cos_theta * local_x - sin_theta * local_y
        rotated_y = sin_theta * local_x + cos_theta * local_y
        
        # Translate to world coordinates
        x = rotated_x + origin.x
        y = rotated_y + origin.y
        
        return x, y

    # ----------------- A* Algorithm -----------------
    def plan_path(self):
        robot_x, robot_y = self.get_robot_pose()
        if robot_x is None:
            self.get_logger().warn("Cannot get robot pose.")
            return

        start = self.world_to_map(robot_x, robot_y)
        goal = self.world_to_map(self.goal_pose.position.x, self.goal_pose.position.y)

        # Debug information
        origin = self.map_info.origin.position
        resolution = self.map_info.resolution
        height, width = self.map.shape
        
        # Calculate map world bounds
        map_min_x = origin.x
        map_max_x = origin.x + width * resolution
        map_min_y = origin.y
        map_max_y = origin.y + height * resolution
        
        self.get_logger().info(f"Robot world pose: ({robot_x:.3f}, {robot_y:.3f})")
        self.get_logger().info(f"Goal world pose: ({self.goal_pose.position.x:.3f}, {self.goal_pose.position.y:.3f})")
        self.get_logger().info(f"Map world bounds: X[{map_min_x:.3f}, {map_max_x:.3f}], Y[{map_min_y:.3f}, {map_max_y:.3f}]")
        self.get_logger().info(f"Start map coords: {start}")
        self.get_logger().info(f"Goal map coords: {goal}")
        
        # Check if start and goal are within map bounds
        height, width = self.map.shape
        if not (0 <= start[0] < width and 0 <= start[1] < height):
            self.get_logger().error(f"Start position {start} is outside map bounds ({width}x{height})")
            return
        if not (0 <= goal[0] < width and 0 <= goal[1] < height):
            self.get_logger().error(f"Goal position {goal} is outside map bounds ({width}x{height})")
            return
            
        # Check if start and goal are in obstacles
        start_cell_value = self.inflated_map[start[1], start[0]]
        goal_cell_value = self.inflated_map[goal[1], goal[0]]
        self.get_logger().info(f"Start cell cost: {start_cell_value} (impassable if >= 255)")
        self.get_logger().info(f"Goal cell cost: {goal_cell_value} (impassable if >= 255)")
        
        if start_cell_value >= 255:
            self.get_logger().error("Start position is in an obstacle!")
            return
        if goal_cell_value >= 255:
            self.get_logger().error("Goal position is in an obstacle!")
            return

        path_indices = self.a_star(start, goal)
        if path_indices is None:
            self.get_logger().warn("Path not found.")
            return

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for mx, my in path_indices:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            wx, wy = self.map_to_world(mx, my)
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Path published with {len(path_indices)} points.")

    def a_star(self, start, goal):
        if self.inflated_map is None:
            self.get_logger().error("Inflated map not available!")
            return None
            
        height, width = self.inflated_map.shape
        neighbors = [(0,1), (1,0), (-1,0), (0,-1), (1,1), (1,-1), (-1,1), (-1,-1)]  # 8-connected

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}

        def heuristic(a, b):
            return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
        def get_cell_cost(x, y):
            """Get the cost of moving through a cell"""
            if not (0 <= x < width and 0 <= y < height):
                return float('inf')  # Out of bounds
            
            cell_value = self.inflated_map[y, x]
            
            if cell_value >= 255:
                return float('inf')  # Impassable obstacle
            else:
                # Base cost (1) + inflation cost
                return 1.0 + (cell_value / 50.0) * 5.0  # Scale inflation cost

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            for dx, dy in neighbors:
                nx, ny = current[0] + dx, current[1] + dy
                
                cell_cost = get_cell_cost(nx, ny)
                if cell_cost == float('inf'):
                    continue
                
                # Diagonal moves cost more
                move_cost = 1.414 if abs(dx) + abs(dy) == 2 else 1.0
                new_cost = g_score[current] + move_cost * cell_cost
                
                if (nx, ny) not in g_score or new_cost < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = new_cost
                    f_score = new_cost + heuristic((nx, ny), goal)
                    heapq.heappush(open_set, (f_score, (nx, ny)))
                    came_from[(nx, ny)] = current
        return None


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
