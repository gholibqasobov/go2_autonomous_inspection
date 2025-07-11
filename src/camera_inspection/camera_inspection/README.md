1. Create map in multifloor dir
```
ros2 run map_provider swtich_map 2
```
2. Start markers in go2_autonomous_inspection
```
ros2 run camera_inspection damaged_objects_rviz_publisher
```
3. Start rviz and add with map also 'MarkerArray' and topic '/damaged_objects_markers'


colcon build --packages-select camera_inspection


source install/setup.bash


ros2 launch realsense2_camera rs_launch.py



Start the server
ros2 run camera_inspection damaged_object_tracker_server

Start inspection
ros2 service call /damaged_object_tracker_server/start_inspection std_srvs/srv/SetBool "{data: true}"

Check status
ros2 service call /damaged_object_tracker_server/get_status std_srvs/srv/Trigger "{}"

Stop inspection
ros2 service call /damaged_object_tracker_server/start_inspection std_srvs/srv/SetBool "{data: false}"

Run demo
ros2 run camera_inspection damaged_object_tracker_client demo 30