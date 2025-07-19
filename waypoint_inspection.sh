#!/bin/bash

# Change directory to home directory
echo "Changing to home directory..."
cd ~


source ~/go2_autonomous_inspection/install/setup.bash
source ~/autonomy_stack_go2/install/setup.bash
sleep 2
cd go2_autonomous_inspection
# Launch RealSense Camera
gnome-terminal -- bash -c "ros2 run camera_inspection damaged_object_tracker_server; exec bash"
sleep 2


# Launch Servo Node
gnome-terminal -- bash -c "ros2 run go2_waypoint_nav camera_servo_server; exec bash"
sleep 2


# Launch Servo Node
gnome-terminal -- bash -c "ros2 run go2_waypoint_nav go2_waypoint_follower_server; exec bash"
sleep 2
