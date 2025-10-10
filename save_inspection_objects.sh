#!/bin/bash

# Change directory to home directory
echo "Changing to home directory..."
cd ~

# Unity Simulation
gnome-terminal -- bash -c "./autonomy_stack_go2/system_simulation.sh; exec bash"
sleep 5

# start map
gnome-terminal -- bash -c "ros2 launch test_map_ws/src/map_provider/launch/map_provider2.launch.py; exec bash"
echo "Changing to go2_autonomous_inspection directory..."
cd ~/go2_autonomous_inspection

# joy publisher
gnome-terminal -- bash -c "ros2 run go2_waypoint_nav waypoint_saver_with_object_id; exec bash"
sleep 1



echo "Started Saving Objects for Inspections"

