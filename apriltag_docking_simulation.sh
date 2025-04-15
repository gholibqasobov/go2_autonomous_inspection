#!/bin/bash

# Change directory to home directory
echo "Changing to home directory..."
cd ~

# Open separate terminals and run the commands

# Camera and AprilTag
echo "Starting RealSense Camera in a new terminal..."
gnome-terminal -- bash -c "ros2 launch realsense2_camera rs_launch.py; exec bash"
sleep 2
echo "Starting AprilTag in a new terminal..."
gnome-terminal -- bash -c "ros2 run apriltag_ros apriltag_node --ros-args \
    -r image_rect:=/camera/camera/color/image_raw \
    -r camera_info:=/camera/camera/color/camera_info \
    --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml; exec bash"
sleep 2
# Robot Simulation
echo "Starting Robot Simulation in a new terminal..."
gnome-terminal -- bash -c "./autonomy_stack_go2/system_simulation.sh; exec bash"
sleep 3

echo "All processes are now running in separate terminals."

