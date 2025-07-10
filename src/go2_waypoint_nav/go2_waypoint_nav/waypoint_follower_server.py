import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Joy
import yaml
import sys
import time


class WaypointFollowerServer(Node):
    
    def __init__(self):
        super().__init__('waypoint_follower_server')

        # self._waypoint_follower_server = 
