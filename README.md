# go2_autonomous_inspection

## Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/gholibqasobov/go2_autonomous_inspection.git
   ```

2. **Navigate to the project directory**:
   ```bash
   cd go2_autonomous_inspection
   
   colcon build --symlink-install
   ```

4. **Source the setup file**:
   ```bash
   source install/setup.bash
   ```

## Usage

### Saving Goal Points
To save goal points, run the following command:
```bash
ros2 run go2_waypoint_nav go2_waypoint_saver
```

### Navigating to Saved Goal Points
In another terminal, after saving the goal points, use:
```bash
ros2 run go2_waypoint_nav go2_waypoint_follower
```

This will navigate the robot to the saved goal points.

### Using move_robot function

In .bashrc file add
```bash
source ~/autonomy_stack_go2/install/setup.bash
source ~/go2_autonomous_inspection/install/setup.bash
```
and source the .bashrc file

Run the following command
```bash
ros2 run go2_apriltag_docking talker
```
