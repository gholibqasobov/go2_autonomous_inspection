# DetectClosest Action Client — Use in Your Code

This guide shows how to call the DetectClosest action from Python code.

It covers two patterns:
- Reuse the provided client class (blocking call)
- Embed an ActionClient in your own node (non‑blocking callbacks)

Prereqs
- Build + source your workspace:
  - `colcon build --symlink-install`
  - `source install/setup.bash`
- Start the server with parameters:
  - `ros2 run camera_inspection damaged_object_tracker_server --ros-args --params-file src/camera_inspection/config/detect_closest_params.yaml`

Option 1 — Reuse the provided client class
- Import the client and make a blocking call that waits for the result.

Example: `examples/call_detect_closest.py`

```python
#!/usr/bin/env python3
import rclpy
from camera_inspection.damaged_object_tracker_client import DetectClosestClient

def main():
    rclpy.init()
    node = DetectClosestClient()
    try:
        # Wait for the action server
        node.get_logger().info('Waiting for DetectClosest server...')
        if not node.wait_for_server(timeout_sec=10.0):
            node.get_logger().error('Server not available')
            return 1

        # Send goal and wait for result
        goal_args = dict(class_name='crack', timeout_s=30.0, confirm_frames=3, require_depth=True)
        result_tuple = node.send_goal(**goal_args)
        if result_tuple is None:
            node.get_logger().warn('Goal was rejected')
            return 2
        status, result = result_tuple
        node.get_logger().info(
            f"success={result.success} reason={result.reason} id={result.saved_id} "
            f"bbox=({result.bbox_x1},{result.bbox_y1},{result.bbox_x2},{result.bbox_y2}) "
            f"world=({result.world_x:.2f},{result.world_y:.2f},{result.world_z:.2f}) status={status}"
        )
        return 0 if result.success else 3
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    raise SystemExit(main())
```

Notes
- The client prints feedback while waiting (closest distance, candidates, camera motion, depth readiness).
- Use larger `timeout_s` if your parameter sequences include multiple poses.

Option 2 — Embed an ActionClient in your node
- Use the async API with callbacks so your node keeps spinning.

Example: `my_package/my_node.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from go2_interfaces.action import DetectClosest

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.client = ActionClient(self, DetectClosest, '/damaged_object_tracker_server/detect_closest')
        self.get_logger().info('Waiting for server...')
        self.client.wait_for_server(10.0)
        self.send_request()

    def send_request(self):
        goal = DetectClosest.Goal()
        goal.class_name = 'crack'
        goal.timeout_s = 30.0
        goal.confirm_frames = 3
        goal.require_depth = True
        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        closest = f"{fb.closest_px_dist:.1f}px" if fb.closest_px_dist >= 0 else 'N/A'
        self.get_logger().info(
            f"feedback: class={fb.active_class} closest={closest} candidates={fb.candidates} "
            f"moving={fb.camera_moving} depth_ready={fb.depth_ready}"
        )

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        res = future.result().result
        self.get_logger().info(
            f"result: success={res.success} reason={res.reason} id={res.saved_id} "
            f"bbox=({res.bbox_x1},{res.bbox_y1},{res.bbox_x2},{res.bbox_y2}) "
            f"world=({res.world_x:.2f},{res.world_y:.2f},{res.world_z:.2f})"
        )


def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Canceling a goal (optional)
- With the provided client: call `cancel_goal()` (e.g., from a timer).
- From your own node: call `goal_handle.cancel_goal_async()`.

Server parameters that affect behavior
- Class/group preset angles: `group_a_*`, `group_b_*` in `src/camera_inspection/config/detect_closest_params.yaml`
- Multi‑step sequences (optional): `camera_sequences_json`, `group_a_sequence_json`, `group_b_sequence_json`, `camera_sequence_repeat`
  - Sequences run before searching; they respect cancel/timeout and stop on success.

Troubleshooting
- Ensure the server is running and reachable at `/damaged_object_tracker_server/detect_closest`.
- Increase `timeout_s` if you configured multi‑pose sequences with long waits.
- Confirm topics/services are active: `ros2 action info /damaged_object_tracker_server/detect_closest`.

Related files
- Client implementation: `src/camera_inspection/camera_inspection/damaged_object_tracker_client.py`
- Server: `src/camera_inspection/camera_inspection/damaged_object_tracker_server.py`
- Params: `src/camera_inspection/config/detect_closest_params.yaml`

