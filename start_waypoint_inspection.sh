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

# A* Path Planner
gnome-terminal -- bash -c "ros2 run go2_nav astar_planner; exec bash"
sleep 3
# Path Follower
gnome-terminal -- bash -c "ros2 run go2_nav path_follower; exec bash"
sleep 1

echo "Started Global Navigation"

