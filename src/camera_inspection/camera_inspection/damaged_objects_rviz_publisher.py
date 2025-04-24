import rclpy
from rclpy.node import Node
import os
import json
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

class DamagedObjectsRvizPublisher(Node):
    def __init__(self):
        super().__init__('damaged_objects_rviz_publisher')
        # Locate JSON in package share directory
        share_dir = get_package_share_directory('camera_inspection')
        self.json_file = os.path.join(share_dir, 'damaged_objects', 'damaged_objects_data.json')
        if not os.path.exists(self.json_file):
            self.get_logger().error(f"damaged_objects_data.json not found at {self.json_file}")
            rclpy.shutdown()
            return

        with open(self.json_file, 'r') as f:
            self.objects_data = json.load(f)

        # Publisher for MarkerArray
        self.publisher = self.create_publisher(MarkerArray, 'damaged_objects_markers', 10)
        # Publish periodically
        self.timer = self.create_timer(1.0, self.publish_markers)

    def publish_markers(self):
        marker_array = MarkerArray()
        for idx, obj in enumerate(self.objects_data):
            # Sphere marker for object position
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'damaged_objects'
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            pos = obj.get('robot_position', {})
            marker.pose.position.x = pos.get('x', 0.0)
            marker.pose.position.y = pos.get('y', 0.0)
            marker.pose.position.z = pos.get('z', 0.0)
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker.lifetime = Duration()
            marker_array.markers.append(marker)

            # Text marker for ID label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'damaged_objects_text'
            text_marker.id = idx + len(self.objects_data)
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = marker.pose
            text_marker.pose.position.z += 0.25
            text_marker.scale.z = 0.2
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"ID: {obj.get('id', idx)}"
            marker_array.markers.append(text_marker)

        self.publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = DamagedObjectsRvizPublisher()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()