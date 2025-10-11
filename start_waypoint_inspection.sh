#!/bin/bash

# Change directory to home directory
echo "Changing to home directory..."
cd ~

# Unity Simulation
gnome-terminal -- bash -c "./autonomy_stack_go2/system_simulation.sh; exec bash"
sleep 5

# start map
gnome-terminal -- bash -c "ros2 launch test_map_ws/src/map_provider/launch/map_provider2.launch.py; exec bash"

# joy publisher
gnome-terminal -- bash -c "ros2 run go2_move go2_vel_by_joy; exec bash"
sleep 1

gnome-terminal -- bash -c "ros2 run go2_waypoint_nav go2_waypoint_follower_server; exec bash"
sleep 1

gnome-terminal -- bash -c "ros2 run go2_waypoint_nav rotate_robot_server; exec bash"
sleep 1

# A* Path Planner
gnome-terminal -- bash -c "ros2 run go2_nav astar_planner; exec bash"
sleep 3
# Path Follower
gnome-terminal -- bash -c "ros2 run go2_nav path_follower; exec bash"
sleep 1


# Servo
gnome-terminal -- bash -c "ros2 run camera_servo camera_servo_node; exec bash"
sleep 1

# Camera
gnome-terminal -- bash -c "ros2 launch realsense2_camera rs_launch.py; exec bash"
sleep 1

cd ~/go2_autonomous_inspection

# Detection
gnome-terminal -- bash -c "ros2 run camera_inspection damaged_object_tracker_server --ros-args --params-file src/camera_inspection/config/detect_closest_params.yaml; exec bash"
sleep 1

echo "Started Global Navigation"

