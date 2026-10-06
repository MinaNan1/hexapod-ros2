"""
path_follower.py
================
ROS 2 node  :  hexapod_path_follower

Loads final_smooth_path_commands.json and drives the hexapod through
every path segment by publishing to:

  /hexapod/body_pose  (geometry_msgs/Twist)
        angular.z  →  yaw / steering command  (rad)
        linear.x/y →  body TX/TY offset       (mm)
        linear.z   →  body TZ / height offset  (mm)
        angular.x  →  roll                     (rad)
        angular.y  →  pitch                    (rad)

  /hexapod/cmd  (std_msgs/String)
        "walk"  / "stop"

Timing
------
Each segment runs for   t = segment_length (m) / WALK_VEL_MS (m/s)
before the node latches the next segment's pose command.
WALK_VEL_MS must match the constant in hexapod_controller.py.

Usage
-----
# default: looks for JSON next to this script
ros2 run hexapod_path_planning hexapod_path_follower

# explicit JSON path via ROS parameter
ros2 run hexapod_path_planning hexapod_path_follower \
    --ros-args -p json_path:=/abs/path/to/final_smooth_path_commands.json

# inside a launch file
Node(
    package='hexapod_path_planning',
    executable='hexapod_path_follower',
    parameters=[{'json_path': '/path/to/final_smooth_path_commands.json'}]
)
"""

import json
import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# ── Must match hexapod_controller.py ─────────────────────────────────────────
#   WALK_VEL_MS = 2 * (coxia/1000) * sin(hip_swing°) / cycle_time
#   With coxia=69.8 mm, hip_swing=25°, step_count=5, timer_hz=20
#   → stride ≈ 0.0590 m / cycle  →  WALK_VEL_MS ≈ 0.059 m/s
WALK_VEL_MS: float = 0.059

# ── Control frequency (Hz) — keep in sync with controller timer ───────────────
CONTROL_HZ: float = 20.0


class HexapodPathFollower(Node):
    """
    Segment-by-segment path executor.

    State machine
    ─────────────
    IDLE  ──(start delay)──▶  WALKING  ──(all segments done)──▶  DONE
                                  │
                          (segment timer expires)
                                  │
                          latch next segment
    """

    def __init__(self) -> None:
        super().__init__('hexapod_path_follower')

        # ── ROS parameter: path to JSON ────────────────────────────────────
        self.declare_parameter('json_path', '')
        self.declare_parameter('start_delay', 1.0)   # seconds before walking

        json_path: str = (
            self.get_parameter('json_path').get_parameter_value().string_value
        )
        self._start_delay: float = (
            self.get_parameter('start_delay').get_parameter_value().double_value
        )

        if not json_path:
            json_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                '..', 'outputs', 'final_smooth_path_commands.json'
            )

        # ── Load JSON ──────────────────────────────────────────────────────
        self._segments: list[dict] = self._load_segments(json_path)
        if not self._segments:
            self.get_logger().error('No segments loaded — node will stay idle.')

        # ── Internal state ─────────────────────────────────────────────────
        self._seg_idx: int       = 0      # index of the active segment
        self._seg_elapsed: float = 0.0    # seconds spent on current segment
        self._seg_duration: float = 0.0   # total time budget for current segment

        self._state: str         = 'IDLE'
        self._idle_elapsed: float = 0.0

        # ── Publishers ─────────────────────────────────────────────────────
        self._pose_pub = self.create_publisher(
            Twist,  '/hexapod/body_pose', 10)
        self._cmd_pub  = self.create_publisher(
            String, '/hexapod/cmd',       10)

        # ── Timer  (20 Hz, same cadence as hexapod_controller) ─────────────
        self._dt: float = 1.0 / CONTROL_HZ
        self._timer     = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'PathFollower ready — {len(self._segments)} segment(s) '
            f'loaded from: {json_path}'
        )
        self.get_logger().info(
            f'Starting in {self._start_delay:.1f} s …  '
            f'(WALK_VEL_MS = {WALK_VEL_MS} m/s)'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # JSON loader
    # ──────────────────────────────────────────────────────────────────────────

    def _load_segments(self, path: str) -> list[dict]:
        try:
            with open(path, 'r') as fh:
                data = json.load(fh)
            if not isinstance(data, list):
                raise ValueError('JSON root must be a list of segment objects.')
            self.get_logger().info(f'Loaded {len(data)} segment(s) from {path}')
            return data
        except FileNotFoundError:
            self.get_logger().error(f'JSON not found: {path}')
        except (json.JSONDecodeError, ValueError) as exc:
            self.get_logger().error(f'JSON parse error: {exc}')
        return []

    # ──────────────────────────────────────────────────────────────────────────
    # Publish helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_pose(self, seg: dict) -> None:
        """
        Map one JSON segment → /hexapod/body_pose Twist.

        JSON fields          →  Twist field
        ─────────────────────────────────────
        tx   (mm)            →  linear.x
        ty   (mm)            →  linear.y
        tz   (mm)            →  linear.z
        roll (rad)           →  angular.x
        pitch(rad)           →  angular.y
        yaw  (rad)  STEERING →  angular.z   ← hexapod_controller uses this
                                               for both leg overlay AND odometry
                                               heading rate
        """
        msg = Twist()
        msg.linear.x  = float(seg.get('tx',    0.0))
        msg.linear.y  = float(seg.get('ty',    0.0))
        msg.linear.z  = float(seg.get('tz',    0.0))
        msg.angular.x = float(seg.get('roll',  0.0))
        msg.angular.y = float(seg.get('pitch', 0.0))
        msg.angular.z = float(seg.get('yaw',   0.0))
        self._pose_pub.publish(msg)

    def _publish_cmd(self, cmd: str) -> None:
        msg       = String()
        msg.data  = cmd
        self._cmd_pub.publish(msg)

    def _publish_zero_pose(self) -> None:
        self._pose_pub.publish(Twist())   # all-zero Twist

    # ──────────────────────────────────────────────────────────────────────────
    # Segment helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _duration_for(self, seg: dict) -> float:
        """
        Time budget for this segment (seconds).
        t = segment_length [m] / forward_walk_speed [m/s]
        Floor at one timer period so zero-length waypoints still get published.
        """
        length = float(seg.get('segment_length', 0.0))
        if length <= 0.0:
            return self._dt
        return max(self._dt, length / WALK_VEL_MS)

    def _activate_segment(self, idx: int) -> None:
        """Latch a segment: publish its pose and reset the elapsed timer."""
        seg                 = self._segments[idx]
        self._seg_elapsed   = 0.0
        self._seg_duration  = self._duration_for(seg)
        self._publish_pose(seg)

        self.get_logger().info(
            f'[{idx + 1}/{len(self._segments)}]  '
            f'heading={seg.get("heading_deg", 0.0):6.1f}°  '
            f'yaw={seg.get("yaw", 0.0):+.4f} rad  '
            f'tx={seg.get("tx", 0.0):+6.2f} mm  '
            f'ty={seg.get("ty", 0.0):+6.2f} mm  '
            f'len={seg.get("segment_length", 0.0):.4f} m  '
            f'→ {self._seg_duration:.2f} s'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # State machine  (called at CONTROL_HZ)
    # ──────────────────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        # ── IDLE: wait for start delay, then kick off walking ─────────────
        if self._state == 'IDLE':
            self._idle_elapsed += self._dt
            if self._idle_elapsed >= self._start_delay and self._segments:
                self._state = 'WALKING'
                self._seg_idx = 0
                self._publish_cmd('walk')
                self._activate_segment(self._seg_idx)
                self.get_logger().info('▶  WALKING — path execution started.')
            return

        # ── DONE: nothing left to do ────────────────────────────────────────
        if self._state == 'DONE':
            return

        # ── WALKING: tick current segment, advance when time is up ────────
        self._seg_elapsed += self._dt

        # Re-publish pose every tick so late-joining subscribers see it
        self._publish_pose(self._segments[self._seg_idx])

        if self._seg_elapsed >= self._seg_duration:
            self._seg_idx += 1

            if self._seg_idx >= len(self._segments):
                # ── Path complete ──────────────────────────────────────────
                self._state = 'DONE'
                self._publish_cmd('stop')
                self._publish_zero_pose()
                self.get_logger().info('■  DONE — all segments executed, robot stopped.')
                self._timer.cancel()
                return

            # Advance to next segment
            self._activate_segment(self._seg_idx)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = HexapodPathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()