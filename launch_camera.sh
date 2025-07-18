#!/bin/bash

# Change directory to home directory
echo "Changing to home directory..."
cd ~


source ~/go2_autonomous_inspection/install/setup.bash
source ~/autonomy_stack_go2/install/setup.bash
sleep 2


# Launch RealSense Camera
gnome-terminal -- bash -c "ros2 launch realsense2_camera rs_launch.py; exec bash"
sleep 2

# Launch Servo Node
gnome-terminal -- bash -c "ros2 run camera_servo camera_servo_node; exec bash"
sleep 2


