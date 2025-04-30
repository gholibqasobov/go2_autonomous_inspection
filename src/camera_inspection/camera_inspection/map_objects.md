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

