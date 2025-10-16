# Usage
## Launch
-   ```bash
    cd ~/autonomy_stack_go2 
    ```
- For Simulaiton:
    ```bash
    ./system_simulation.sh
    ```
    For Real Robot: 
    ```bash
    ./system_real_robot.sh
    ```
    
### Saving Goal Points
Goal Point Creation:
1. Run goal point saver node:
    ```bash
    ros2 run go2_waypoint_nav go2_waypoint_saver
    ```
2. Enter the goal points amount
3. In __rviz__ select "Add a new tool" (+ button), add "SetGoal" and click "Ok". "2D Goal Pose" button appears in the toolbar.
4. Select "2D Goal Pose" 
5. On the map, point to the place to set as waypoint
6. Repeat steps __4__ and __5__ the number of times you entered in step __2__ 

Note: _Goal points should be located close to each other, so the robot could find the path._

### Navigating to Saved Goal Points
After saving goal points:
```bash
ros2 run go2_waypoint_nav go2_waypoint_follower
```

The robot will navigate to the saved goal points.


```bash
ros2 action send_goal /camera_servo_action camera_servo_interfaces/action/CameraServo "{start: true}" --feedback
```





Main node to run the inspection:
```bash
ros2 run go2_waypoint_nav inspection_client_main 
```

Need to start:

- Camera Inspection Server
- Waypoint Follower Server
- Rotate Robot Server