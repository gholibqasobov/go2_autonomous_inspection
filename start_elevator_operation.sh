#!/bin/bash

# Change directory to home directory
echo "Changing to home directory..."
cd ~


source ~/go2_autonomous_inspection/install/setup.bash
sleep 2

# Push Elevator Button Server
gnome-terminal -- bash -c "ros2 run roarm_elevator_controller push_elevator_button_server; exec bash"
sleep 2

# AprilTag Docking Server
gnome-terminal -- bash -c "ros2 run go2_apriltag_docking apriltag_docking_server; exec bash"
sleep 2

# Move Robot Server
gnome-terminal -- bash -c "ros2 run go2_move go2_move_server; exec bash"
sleep 2

# Elevator Operation Client
gnome-terminal -- bash -c "ros2 run elevator_controller elevator_controller_client; exec bash"
