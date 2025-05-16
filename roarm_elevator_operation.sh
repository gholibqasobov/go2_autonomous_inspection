#!/bin/bash

# Change directory to home directory
echo "Changing to home directory..."
cd ~

# RoArm Manipulator
source ~/roarm_ws_em0/install/setup.bash
sudo chmod 666 /dev/ttyUSB0
sleep 3
gnome-terminal -- bash -c "ros2 run roarm_driver roarm_driver; exec bash"
sleep 1
# Robot Simulation
gnome-terminal -- bash -c "ros2 launch roarm_moveit_cmd command_control.launch.py; exec bash"
sleep 3
# Init Move Point Node
gnome-terminal -- bash -c "ros2 run roarm_moveit_cmd movepointcmd; exec bash"
sleep 1

echo "Starting Push-Button server"

gnome-terminal -- bash -c "ros2 run roarm_elevator_controller move_point_client 0.15 0.0 0.4; exec bash"

sleep 1
#gnome-terminal -- bash -c "ros2 run roarm_elevator_controller push_elevator_button_server; exec bash"

