#!/usr/bin/env python3

import argparse
import sys
from typing import Optional
import os
import json
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from go2_interfaces.action import DetectClosest


class DetectClosestClient(Node):
    """Action client for the DetectClosest server (damaged_object_tracker_client).

    Usage examples:
      ros2 run camera_inspection damaged_object_tracker_client --class crack
      ros2 run camera_inspection damaged_object_tracker_client --class crack \
          --timeout 30 --confirm 3 --require-depth
    """

    def __init__(self):
        super().__init__('damaged_object_tracker_client')
        self._client = ActionClient(self, DetectClosest, '/damaged_object_tracker_server/detect_closest')
        self._goal_handle = None

    def wait_for_server(self, timeout_sec: Optional[float] = None) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def feedback_cb(self, feedback_msg: DetectClosest.Feedback):
        fb = feedback_msg.feedback
        closest = f"{fb.closest_px_dist:.1f}px" if fb.closest_px_dist >= 0 else "N/A"
        self.get_logger().info(
            f"[feedback] class={fb.active_class} closest={closest} "
            f"candidates={fb.candidates} camera_moving={fb.camera_moving} depth_ready={fb.depth_ready}"
        )

    def send_goal(self, class_name: str, timeout_s: float, confirm_frames: int, require_depth: bool):
        goal = DetectClosest.Goal()
        goal.class_name = class_name
        goal.timeout_s = float(timeout_s)
        goal.confirm_frames = int(confirm_frames)
        goal.require_depth = bool(require_depth)

        self.get_logger().info(
            f"Sending goal: class='{class_name}', timeout={timeout_s}s, "
            f"confirm_frames={confirm_frames}, require_depth={require_depth}"
        )

        send_future = self._client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        self._goal_handle = send_future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server')
            return None
        self.get_logger().info('Goal accepted')

        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        status = result_future.result().status
        return status, result

    def cancel_goal(self) -> bool:
        if not self._goal_handle:
            return False
        cancel_future = self._goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)
        return True

    # ----- logging helpers -----
    @staticmethod
    def _ensure_parent_dir(path: str):
        try:
            parent = os.path.dirname(path)
            if parent:
                os.makedirs(parent, exist_ok=True)
        except Exception:
            pass

    def log_run(self, log_file: str, *,
                class_name: str,
                success: bool,
                reason: str,
                result: Optional[DetectClosest.Result],
                params: dict):
        """Append one JSON line describing this run to log_file.

        Uses Option B location by default (under src/) but allows override via CLI.
        """
        record = {
            'client_ts': float(time.time()),
            'action_name': '/damaged_object_tracker_server/detect_closest',
            'class_name': class_name,
            'success': bool(success),
            'reason': str(reason) if reason is not None else '',
            'params': params or {},
        }
        if result is not None:
            try:
                record.update({
                    'saved_id': int(result.saved_id),
                    'bbox': [int(result.bbox_x1), int(result.bbox_y1), int(result.bbox_x2), int(result.bbox_y2)],
                    'world': [float(result.world_x), float(result.world_y), float(result.world_z)],
                    'image_file': str(result.image_file),
                    'server_ts': float(result.timestamp),
                })
            except Exception:
                # Best-effort if fields are missing or types differ
                pass
        self._ensure_parent_dir(log_file)
        try:
            with open(log_file, 'a') as f:
                f.write(json.dumps(record) + "\n")
        except Exception as e:
            self.get_logger().warn(f"Failed to write log to '{log_file}': {e}")


def parse_args(argv):
    parser = argparse.ArgumentParser(description='DetectClosest action client')
    parser.add_argument('--class', dest='class_name', required=True, help='Target class name (e.g., crack)')
    parser.add_argument('--timeout', type=float, default=20.0, help='Timeout seconds')
    parser.add_argument('--confirm', type=int, default=3, help='Confirm frames')
    parser.add_argument('--require-depth', action='store_true', help='Require depth measurement')
    parser.add_argument('--cancel-after', type=float, default=None, help='Cancel after N seconds (for testing)')
    parser.add_argument('--log-file', type=str,
                        default=os.path.join('src', 'camera_inspection', 'damaged_objects', 'detect_closest_runs.jsonl'),
                        help='Path to append JSONL logs (default under src/)')
    return parser.parse_args(argv)


def main(argv=None):
    rclpy.init(args=argv)
    args = parse_args(sys.argv[1:])

    node = DetectClosestClient()
    try:
        node.get_logger().info('Waiting for DetectClosest action server...')
        if not node.wait_for_server(timeout_sec=10.0):
            node.get_logger().error('Server not available')
            # Log rejected-equivalent outcome (no server)
            node.log_run(
                args.log_file,
                class_name=args.class_name,
                success=False,
                reason='server_unavailable',
                result=None,
                params={
                    'timeout_s': float(args.timeout),
                    'confirm_frames': int(args.confirm),
                    'require_depth': bool(args.require_depth),
                },
            )
            return 1

        # Optional timed cancel
        timer = None
        if args.cancel_after is not None and args.cancel_after > 0.0:
            def _cancel():
                node.get_logger().warn('Cancel timer fired — sending cancel request')
                node.cancel_goal()
            timer = node.create_timer(float(args.cancel_after), _cancel)

        result_tuple = node.send_goal(
            class_name=args.class_name,
            timeout_s=args.timeout,
            confirm_frames=args.confirm,
            require_depth=args.require_depth,
        )
        if result_tuple is None:
            # Goal was rejected
            node.log_run(
                args.log_file,
                class_name=args.class_name,
                success=False,
                reason='rejected',
                result=None,
                params={
                    'timeout_s': float(args.timeout),
                    'confirm_frames': int(args.confirm),
                    'require_depth': bool(args.require_depth),
                },
            )
            return 2
        status, result = result_tuple
        node.get_logger().info(
            f"Result: success={result.success} reason='{result.reason}' id={result.saved_id} "
            f"bbox=({result.bbox_x1},{result.bbox_y1},{result.bbox_x2},{result.bbox_y2}) "
            f"world=({result.world_x:.2f},{result.world_y:.2f},{result.world_z:.2f}) "
            f"image='{result.image_file}' ts={result.timestamp:.3f} status={status}"
        )
        # Log success/timeout/canceled outcomes
        node.log_run(
            args.log_file,
            class_name=args.class_name,
            success=bool(result.success),
            reason=str(result.reason),
            result=result,
            params={
                'timeout_s': float(args.timeout),
                'confirm_frames': int(args.confirm),
                'require_depth': bool(args.require_depth),
            },
        )
        if timer:
            node.destroy_timer(timer)
        return 0 if result.success else 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
