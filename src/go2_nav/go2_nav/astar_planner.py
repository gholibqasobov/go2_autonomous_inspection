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
        self.get_logger().info(f"Map received: {msg.info.width}x{msg.info.height}, resolution: {msg.info.resolution:.3f}m")

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
                'map', 'base_link', rclpy.time.Time())
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
        mx = int((x - origin.x) / resolution)
        my = int((y - origin.y) / resolution)
        return mx, my

    def map_to_world(self, mx, my):
        """Convert map grid indices to world coordinates"""
        origin = self.map_info.origin.position
        resolution = self.map_info.resolution
        x = mx * resolution + origin.x
        y = my * resolution + origin.y
        return x, y

    # ----------------- A* Algorithm -----------------
    def plan_path(self):
        robot_x, robot_y = self.get_robot_pose()
        if robot_x is None:
            self.get_logger().warn("Cannot get robot pose.")
            return

        start = self.world_to_map(robot_x, robot_y)
        goal = self.world_to_map(self.goal_pose.position.x, self.goal_pose.position.y)

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
        height, width = self.map.shape
        neighbors = [(0,1), (1,0), (-1,0), (0,-1)]

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}

        def heuristic(a, b):
            return np.linalg.norm(np.array(a) - np.array(b))

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
                if 0 <= nx < width and 0 <= ny < height:
                    if self.map[ny][nx] > 50:  # obstacle
                        continue
                    new_cost = g_score[current] + 1
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
