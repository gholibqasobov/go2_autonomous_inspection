#!/bin/bash

# Change directory to home directory
echo "Changing to home directory..."
cd ~

# Initializing servers
gnome-terminal -- bash -c "ros2 run go2_apriltag_docking apriltag_docking_server; exec bash"
sleep 1

gnome-terminal -- bash -c "ros2 run roarm_elevator_controller push_elevator_button_server; exec bash"
sleep 1
# Init Move Point Node
gnome-terminal -- bash -c "ros2 run go2_move go2_move_server; exec bash"
sleep 1

gnome-terminal -- bash -c "ros2 run go2_apriltag_docking apriltag_stop_server; exec bash"

sleep 1

echo "Initialized all the servers"

