# DetectClosest Action — Usage & Testing Guide

This guide explains how to run, control, and test the DetectClosest action server provided by `camera_inspection`.

The action finds exactly one object of a commanded class, selects the track whose bounding box center is closest to the image center, saves it using the existing JSON/image pipeline, and then completes.

## Prerequisites
- ROS 2 Humble environment sourced:
  - `source ~/autonomy_stack_go2/install/setup.bash && source install/setup.bash`
- Camera topics available:
  - Color: `/camera/camera/color/image_raw`
  - Depth: `/camera/camera/depth/image_rect_raw`
  - Servo angles: `/servo_angles` (string like `"1000 2007"`)
  - TF: `/tf`
- Build dependencies for the detector/tracker installed (Ultralytics YOLO, DeepSORT runtime, OpenCV, cv_bridge).

## Build
From the workspace root (`go2_autonomous_inspection`):

- `colcon build --symlink-install`
- `source install/setup.bash`

Tip: If you just added the action, ensure `go2_interfaces` is built before `camera_inspection` so the action type is generated.

## Run the Server
- `ros2 run camera_inspection damaged_object_tracker_server`

The action server is created at:
- Name: `~/detect_closest`
- Fully qualified: `/damaged_object_tracker_server/detect_closest`

Verify it is available:
- `ros2 action list`
- `ros2 action info /damaged_object_tracker_server/detect_closest`

## Send an Action Goal
Find one object of class `crack`, with 20s timeout, 3-frame stability requirement, depth optional:

- `ros2 action send_goal /damaged_object_tracker_server/detect_closest go2_interfaces/action/DetectClosest "{class_name: 'crack', timeout_s: 20.0, confirm_frames: 3, require_depth: false}"`

Require valid depth (a 3D/world position must be computed):

- `ros2 action send_goal /damaged_object_tracker_server/detect_closest go2_interfaces/action/DetectClosest "{class_name: 'crack', timeout_s: 30.0, confirm_frames: 3, require_depth: true}"`

Notes
- The action auto-starts inspection if it was idle and auto-stops if it started it.
- Motion gating applies (the server skips detection while the camera is moving).

## What the Server Does
- Gates detections to the commanded `class_name` while the action is active.
- Computes each candidate’s bbox center and distance to image center, selects the smallest distance.
- Requires the same closest track to persist for `confirm_frames` frames, rejects duplicates, and acknowledges `require_depth`.
- Saves exactly one object (full-frame image + JSON record) and completes the action with a success result.

## Monitoring
- Feedback (published at ~5 Hz):
  - `active_class`, `closest_px_dist`, `candidates`, `camera_moving`, `depth_ready`.
- Annotated image:
  - Topic: `/annotated_image`
  - Use `rqt_image_view` to view; overlay shows `TARGET=<class>` and `Closest(px)`.
- General status (optional):
  - `ros2 service call /damaged_object_tracker_server/get_status std_srvs/srv/Trigger {}`

## Cancel / Preempt
- Cancel the current goal:
  - `ros2 action send_goal --cancel /damaged_object_tracker_server/detect_closest go2_interfaces/action/DetectClosest "{class_name: 'crack'}"`
- Only one session runs at a time; additional goals are rejected until the current goal completes or is canceled.

## Result & Outputs
On success, the action returns:
- `success=true`, `reason='saved'`
- `saved_id` (track ID), `bbox_{x1,y1,x2,y2}`
- `world_x/y/z` (0 if not available), `image_file`, `timestamp`

Saved artifacts:
- JSON: `src/camera_inspection/damaged_objects/damaged_objects_data.json`
- Image: `src/camera_inspection/damaged_objects/object_<id>_<timestamp>.jpg`

## Troubleshooting
- Action not visible:
  - Rebuild and re-source: `colcon build --symlink-install && source install/setup.bash`
  - Check: `ros2 action list`
- No candidates:
  - Verify `class_name` matches YOLO’s class names (see `self.model.names`).
  - Confirm topics match code paths (`/camera/camera/color/image_raw`, etc.).
- Depth required but missing:
  - Ensure depth image is publishing and intrinsics in the server match your camera.
- Camera “MOVING” shown continuously:
  - Reduce motion or adjust thresholds in the server (`STATIONARY_THRESH`, `DIFF_THRESH`, servo thresholds).

## Quick Command Reference
- Run server:
  - `ros2 run camera_inspection damaged_object_tracker_server`
- Info:
  - `ros2 action info /damaged_object_tracker_server/detect_closest`
- Goal:
  - `ros2 action send_goal /damaged_object_tracker_server/detect_closest go2_interfaces/action/DetectClosest "{class_name: 'crack', timeout_s: 20.0, confirm_frames: 3, require_depth: false}"`
- Cancel:
  - `ros2 action send_goal --cancel /damaged_object_tracker_server/detect_closest go2_interfaces/action/DetectClosest "{class_name: 'crack'}"`
- Check outputs:
  - JSON in `src/camera_inspection/damaged_objects/damaged_objects_data.json`
  - Images in `src/camera_inspection/damaged_objects/`

