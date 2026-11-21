# go2_autonomous_inspection

> **Elevator study release (v1.0.0).** This version is the software used in the
> real-world experiments of *"Autonomous Elevator Interaction and Multi-Floor
> Navigation for Quadruped Robots Using Vision and Fiducial-Guided Manipulation"*.


## Prerequisites
   - Intel CPU required
   - [Ubuntu 22.04 LTS](https://releases.ubuntu.com/jammy/)
   - Install [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
   - Install [autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2?tab=readme-ov-file) repository. Follow the instructions for ROS2 Humble on External Computer
   - Install [roarm_ws_em0](https://github.com/waveshareteam/roarm_ws_em0) repository

## Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/gholibqasobov/go2_autonomous_inspection.git
   ```
2. Source __autonomy_stack_go2__ by adding it to your __~/.bashrc__ file:
   ```bash
   gedit ~/.bashrc
   ```
   Add to the end of the file:
   ```bash
   source ~/autonomy_stack_go2/install/setup.bash
   source ~/go2_autonomous_inspection/install/setup.bash
   ```
3. Source the __~/.bashrc__ file:
   ```bash
   source ~/.bashrc
   ```
   Note: _In the first run of the command, ```bash: /home/$USER/go2_autonomous_inspection/install/setup.bash: No such file or directory``` warning will appear._

4. **Navigate to the project directory**:
   ```bash
   cd ~/go2_autonomous_inspection
   
   colcon build --symlink-install
   ```




## Software dependencies
| Dependency | Used for |
|---|---|
| [autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2) (branch `foxy-humble`) | **Required to build and run.** `go2_apriltag_docking` and `go2_move` depend on its `unitree_api` and `go2_sport_api` packages. It also provides the Go2 locomotion bridge (`/joy` → Unitree sport API) started by `system_real_robot.sh`, plus SLAM and navigation. |
| [roarm_ws_em0](https://github.com/waveshareteam/roarm_ws_em0) | RoArm-M2-S driver (`roarm_driver`) and the MoveIt 2 point-command service (`roarm_moveit_cmd movepointcmd`) called by the button-pushing server. |
| [apriltag_ros](https://github.com/christianrauch/apriltag_ros) | AprilTag (tag36h11) detection and pose estimation in the camera frame, published on `/tf`. |
| [realsense-ros](https://github.com/IntelRealSense/realsense-ros) | RealSense D435 driver (`realsense2_camera`). |

Clone `autonomy_stack_go2` and `roarm_ws_em0` into your home directory (`~/autonomy_stack_go2`, `~/roarm_ws_em0`): the launch scripts expect them there.

## Fiducial marker setup
- Markers are AprilTags from the **tag36h11** family, attached next to the external call panel and the in-cabin floor-selection panel.
- Every step of the elevator sequence looks for the TF frame **`elevator_docking`**, so `apriltag_ros` must publish the marker under this frame name.
- The button positions relative to the marker were measured offline. They are hard-coded as vertical offsets (in metres) in `ELEVATOR_BUTTON_OFFSETS` in `src/roarm_elevator_controller/roarm_elevator_controller/push_elevator_button_server.py` (`101` = call/up, `1`, `2`, ... = floor buttons). The camera-to-robot transform is hard-coded in the same file. Re-measure both for a different elevator or mounting.

## Running the elevator task
Run each script from the repository root, in this order. Each one opens its nodes in separate `gnome-terminal` windows.

1. **Camera, AprilTag detection, and Go2 base interface**
   ```bash
   ./apriltag_docking_real_robot.sh
   ```
   Starts `realsense2_camera`, `apriltag_ros` and `~/autonomy_stack_go2/system_real_robot.sh`.
2. **Manipulator**
   ```bash
   ./roarm_elevator_operation.sh
   ```
   Starts the RoArm driver, MoveIt 2 command control and the `movepointcmd` service, then moves the arm to its home pose `(0.3, 0.0, 0.3)`. Uses `sudo` to open `/dev/ttyUSB0`.
3. **Action servers**
   ```bash
   ./launch_servers.sh
   ```
   Starts `apriltag_docking_server`, `push_elevator_button_server`, `go2_move_server` and `apriltag_stop_server`.
4. **Elevator task controller.** Place the robot in the hallway facing the external call panel, with the marker in view. Then run:
   ```bash
   ros2 run elevator_controller elevator_controller_client
   ```


## Repository contents
| Package | Role in the elevator study |
|---|---|
| `elevator_controller` | Task-sequencing client (above) and stand-alone test clients for each action |
| `go2_apriltag_docking` | Marker-based docking server and exit server (`apriltag_stop_server`) |
| `roarm_elevator_controller` | Button-pushing action server for the RoArm-M2-S |
| `go2_move` | Relative-motion action server for the Go2 base |
| `go2_interfaces` | Action definitions (`AprilTagDocking`, `PushElevatorButton`, `MoveToGoal`, `AprilTagStop`) |
| `camera_manipulator` | ESP32 firmware for the pan-tilt camera mount |
| `camera_inspection`, `go2_waypoint_nav` | Separate inspection work; not used in the elevator sequence |

## Citation
If you use this software, please cite it using [`CITATION.cff`](CITATION.cff) (GitHub: *Cite this repository*) or the Zenodo record for this release.

## License
Apache License 2.0. See [LICENSE](LICENSE). Third-party dependencies listed above are distributed under their own terms and are not included in this repository.

