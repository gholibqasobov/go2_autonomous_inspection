#!/bin/bash

# Change directory to home directory
echo "Changing to home directory..."
cd ~

# Open separate terminals and run the commands

# Camera and AprilTag
echo "Starting RealSense Camera in a new terminal..."
gnome-terminal -- bash -c "ros2 launch realsense2_camera rs_launch.py; exec bash"
sleep 1
echo "Starting AprilTag in a new terminal..."
gnome-terminal -- bash -c "ros2 launch apriltag_ros camera_36h11.launch.yml; exec bash"
sleep 1
# Robot Simulation
echo "Starting Robot Simulation in a new terminal..."
gnome-terminal -- bash -c "./autonomy_stack_go2/system_simulation.sh; exec bash"
sleep 5
# AprilTag Docking
echo "Starting AprilTag Docking in a new terminal..."
gnome-terminal -- bash -c "ros2 run go2_apriltag_docking apriltag_docking_controller; exec bash"

echo "All processes are now running in separate terminals."

