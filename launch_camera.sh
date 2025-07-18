ros2 run camera_servo camera_servo_node

ros2 launch realsense2_camera rs_launch.py

ros2 run camera_inspection damaged_object_tracker_server

ros2 run go2_waypoint_nav camera_servo_server

ros2 run go2_waypoint_nav waypoint_client

ros2 run go2_waypoint_nav go2_waypoint_follower
