# go2_autonomous_inspection

## Prerequisites
   - Intel CPU required
   - [Ubuntu 22.04 LTS](https://releases.ubuntu.com/jammy/)
   - Install [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
   - Install [autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2?tab=readme-ov-file) repository. Follow the instructions for ROS2 Humble on External Computer
   - Install [raorm_ws_em0](https://github.com/waveshareteam/roarm_ws_em0) repository

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


## Usage


## FAQ

- If colcon build of [autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2?tab=readme-ov-file) __failed__ with 
   ```
   autonomy_stack_go2/src/route_planner/far_planner/include/far_planner/utility.h:58:10: fatal error: tf2_sensor_msgs/tf2_sensor_msgs.h: No such file or directory 
   ```
   1. In the _autonomy_stack_go2/src/route_planner/far_planner/include/far_planner/utility.h_ file replace
      ``` 
      #include <tf2_sensor_msgs/tf2_sensor_msgs.h>
      ``` 
      to 

      ```
      #include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
      ```
   2. Download __package.xml__ and __CmakeLists.txt__ from [go2_autonomous_inspection/extras/](https://github.com/gholibqasobov/go2_autonomous_inspection/extras/) and replace with __package.xml__ and __CmakeLists.txt__ in _autonomy_stack_go2/src/route_planner/far_planner/_
   3. Install Dependencies:
      ```bash
      sudo apt install ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-sensor-msgs
      ```