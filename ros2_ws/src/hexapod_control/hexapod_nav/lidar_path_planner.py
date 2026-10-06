"""
lidar_path_planner.py
=====================
ROS2 node — plans A* paths on the live occupancy grid built by lidar_mapper,
then drives the hexapod along the planned path.

Subscribes:
    /map        (nav_msgs/OccupancyGrid)        — live 2D map from lidar_mapper
    /goal_pose  (geometry_msgs/PoseStamped)      — goal from RViz "2D Goal Pose"

Publishes:
    /hexapod/body_pose  (geometry_msgs/Twist)    — motion commands
    /hexapod/cmd        (std_msgs/String)        — "walk" / "stop"
    /planned_path       (nav_msgs/Path)          — RViz visualization

Uses the A* planner and path smoother from hexapod_path_planning_final/.

Usage:
    ros2 run hexapod_control lidar_path_planner
    ros2 run hexapod_control lidar_path_planner --ros-args \
        -p planning_module_path:=/path/to/hexapod_path_planning
"""

import math
import os
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String

# ── Import path planning modules from hexapod_path_planning_final ────────────
# Uses sys.path — the default path works for the project layout.
# Override via ROS parameter 'planning_module_path' if needed.
def _find_planning_path():
    _rel = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', '..', '..', '..', '..',
        'hexapod_path_planning_final', 'hexapod_path_planning'))
    candidates = [
        os.environ.get('HEXAPOD_PLANNING_PATH', ''),
        _rel,
        '/hexapodd/hexapod_path_planning_final/hexapod_path_planning',
        os.path.expanduser('~/hexapod-ros2/hexapod_path_planning_final/hexapod_path_planning'),
    ]
    for p in candidates:
        if p and os.path.isdir(p):
            return p
    return candidates[2]

_PLANNING_PATH = _find_planning_path()
sys.path.insert(0, _PLANNING_PATH)

try:
    from map_creation.map_inflator  import MapInflator
    from path_finding.astar_planner import AStarPlanner
    from path_finding.path_smoother import PathSmoother
    _PLANNING_AVAILABLE = True
except ImportError:
    _PLANNING_AVAILABLE = False


# ── Walk speed — must match hexapod_controller.py ────────────────────────────
WALK_VEL_MS = 0.059     # m/s  (from hexapod_controller.py)
CONTROL_HZ  = 20.0      # Hz


class LidarPathPlanner(Node):

    def __init__(self):
        super().__init__('lidar_path_planner')

        if not _PLANNING_AVAILABLE:
            self.get_logger().error(
                'Could not import path planning modules! '
                'Make sure hexapod_path_planning_final/ exists. '
                'Set param planning_module_path if in a different location.')
            raise SystemExit(1)

        # ── ROS2 Parameters ──────────────────────────────────────────
        self.declare_parameter('robot_radius_cells',  2)
        self.declare_parameter('rdp_epsilon',         2.0)
        self.declare_parameter('step_speed_mm',      10.0)
        self.declare_parameter('max_yaw_rad',         0.3)
        self.declare_parameter('replan_on_map_change', True)

        self.robot_radius = self.get_parameter('robot_radius_cells').value
        self.rdp_eps      = self.get_parameter('rdp_epsilon').value
        self.step_speed   = self.get_parameter('step_speed_mm').value
        self.max_yaw      = self.get_parameter('max_yaw_rad').value
        self.auto_replan  = self.get_parameter('replan_on_map_change').value

        # ── Planning helpers ─────────────────────────────────────────
        self.inflator = MapInflator(robot_radius_cells=self.robot_radius)
        self.smoother = PathSmoother()

        # ── Map state ────────────────────────────────────────────────
        self.current_grid    = None     # numpy 2D array (0=free, 1=obstacle)
        self.map_resolution  = 0.05
        self.map_origin_x    = -5.0
        self.map_origin_y    = -5.0
        self.map_width       = 200
        self.map_height      = 200
        self._map_version    = 0        # incremented on each new map

        # ── Robot simulated position (world metres) ──────────────────
        self._robot_x       = 0.0
        self._robot_y       = 0.0
        self._robot_heading = 0.0       # rad

        # ── Path following state ─────────────────────────────────────
        self._commands       = []       # list of body-pose command dicts
        self._cmd_idx        = 0
        self._seg_elapsed    = 0.0
        self._seg_duration   = 0.0
        self._state          = 'IDLE'   # IDLE | WALKING | DONE
        self._goal_grid      = None     # (row, col) of current goal
        self._planned_map_v  = -1       # map version when path was planned

        # ── ROS2 Publishers ──────────────────────────────────────────
        self.body_pub = self.create_publisher(Twist,  '/hexapod/body_pose', 10)
        self.cmd_pub  = self.create_publisher(String, '/hexapod/cmd',       10)
        self.path_pub = self.create_publisher(Path,   '/planned_path',     10)

        # ── ROS2 Subscribers ─────────────────────────────────────────
        self.create_subscription(
            OccupancyGrid, '/map', self._on_map, 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self._on_goal, 10)

        # ── Control loop — 20 Hz ─────────────────────────────────────
        self._dt = 1.0 / CONTROL_HZ
        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            'LidarPathPlanner ready — waiting for /map and /goal_pose')

    # ──────────────────────────────────────────────────────────────────
    # Map callback
    # ──────────────────────────────────────────────────────────────────

    def _on_map(self, msg: OccupancyGrid):
        """Convert OccupancyGrid to numpy array for A*."""
        self.map_resolution = msg.info.resolution
        self.map_width      = msg.info.width
        self.map_height     = msg.info.height
        self.map_origin_x   = msg.info.origin.position.x
        self.map_origin_y   = msg.info.origin.position.y

        # Convert OccupancyGrid data → numpy
        raw = np.array(msg.data, dtype=np.int8).reshape(
            (self.map_height, self.map_width))

        # For A*:  0=free, 1=obstacle
        # OccupancyGrid: -1=unknown, 0=free, 1-100=occupied
        grid = np.zeros_like(raw, dtype=np.int8)
        grid[raw == -1] = 1     # treat unknown as obstacle
        grid[raw > 30]  = 1     # treat occupied as obstacle

        self.current_grid = grid
        self._map_version += 1

    # ──────────────────────────────────────────────────────────────────
    # Goal callback
    # ──────────────────────────────────────────────────────────────────

    def _on_goal(self, msg: PoseStamped):
        """Receive goal from RViz, plan A* path."""
        gx = msg.pose.position.x
        gy = msg.pose.position.y

        self.get_logger().info(f'Goal received: world({gx:.2f}, {gy:.2f})')

        if self.current_grid is None:
            self.get_logger().error('No map received yet — cannot plan.')
            return

        # Convert world → grid
        goal_col = int((gx - self.map_origin_x) / self.map_resolution)
        goal_row = int((gy - self.map_origin_y) / self.map_resolution)

        # Bounds check
        if not (0 <= goal_row < self.map_height and
                0 <= goal_col < self.map_width):
            self.get_logger().error(
                f'Goal ({goal_row}, {goal_col}) out of grid bounds.')
            return

        self._goal_grid = (goal_row, goal_col)
        self._plan_path()

    # ──────────────────────────────────────────────────────────────────
    # Path planning
    # ──────────────────────────────────────────────────────────────────

    def _plan_path(self):
        """Run A* from robot position to goal on current map."""
        if self.current_grid is None or self._goal_grid is None:
            return

        # Robot position → grid coords
        start_col = int((self._robot_x - self.map_origin_x) / self.map_resolution)
        start_row = int((self._robot_y - self.map_origin_y) / self.map_resolution)

        # Clamp to grid bounds
        start_col = max(0, min(start_col, self.map_width  - 1))
        start_row = max(0, min(start_row, self.map_height - 1))

        start = (start_row, start_col)
        goal  = self._goal_grid

        self.get_logger().info(
            f'Planning: grid {start} → {goal}')

        # Inflate obstacles for robot body
        try:
            inflated = self.inflator.inflate(self.current_grid)
        except Exception as e:
            self.get_logger().error(f'Inflation failed: {e}')
            inflated = self.current_grid

        # Check start/goal validity
        if inflated[start[0], start[1]] == 1:
            self.get_logger().warn(
                f'Start {start} is in obstacle — clearing a small area around robot')
            # Clear a small area around robot so planning can start
            r, c = start
            for dr in range(-3, 4):
                for dc in range(-3, 4):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < self.map_height and 0 <= nc < self.map_width:
                        inflated[nr, nc] = 0

        if inflated[goal[0], goal[1]] == 1:
            self.get_logger().error(
                f'Goal {goal} is inside an obstacle. '
                'Try a different goal position.')
            return

        # Run A*
        planner = AStarPlanner(inflated, start, goal)
        result  = planner.plan()

        if not result.found:
            self.get_logger().error(
                f'No path found from {start} to {goal}. '
                f'Explored {result.explored_count} cells.')
            return

        # Smooth path
        rdp_path    = self.smoother.rdp_simplify(result.path, epsilon=self.rdp_eps)
        smooth_path = self.smoother.interpolate(rdp_path, spacing=5.0)

        self.get_logger().info(
            f'Path found: {len(result.path)} raw → '
            f'{len(smooth_path)} smooth waypoints  '
            f'(cost={result.cost:.1f})')

        # Convert grid path → world coordinates
        world_path = [
            (self.map_origin_x + col * self.map_resolution,
             self.map_origin_y + row * self.map_resolution)
            for row, col in smooth_path
        ]

        # Generate body-pose commands
        self._commands = self._path_to_commands(world_path)
        self._cmd_idx       = 0
        self._seg_elapsed   = 0.0
        self._planned_map_v = self._map_version

        # Publish path for RViz
        self._publish_path_viz(world_path)

        # Start walking
        if self._commands:
            self._state = 'WALKING'
            self._seg_duration = self._duration_for(self._commands[0])
            cmd_msg = String()
            cmd_msg.data = 'walk'
            self.cmd_pub.publish(cmd_msg)
            self.get_logger().info(
                f'▶ WALKING — {len(self._commands)} segments')

    # ──────────────────────────────────────────────────────────────────
    # Path → body_pose commands  (from PathConverter logic)
    # ──────────────────────────────────────────────────────────────────

    def _path_to_commands(self, world_path):
        """Convert world-coordinate path to hexapod body_pose commands."""
        if len(world_path) < 2:
            return []

        commands = []
        current_yaw = self._robot_heading

        for i in range(len(world_path) - 1):
            x0, y0 = world_path[i]
            x1, y1 = world_path[i + 1]
            dx, dy = x1 - x0, y1 - y0
            seg_len = math.sqrt(dx * dx + dy * dy)

            if seg_len < 1e-6:
                continue

            target_yaw = math.atan2(dy, dx)
            yaw_error  = self._angle_diff(target_yaw, current_yaw)
            yaw_cmd    = max(-self.max_yaw, min(self.max_yaw, yaw_error))
            current_yaw += yaw_cmd

            if abs(yaw_error) < 0.3:
                tx = self.step_speed * math.cos(current_yaw)
                ty = self.step_speed * math.sin(current_yaw)
            else:
                tx = ty = 0.0       # rotate in place first

            commands.append({
                'tx':             round(tx, 4),
                'ty':             round(ty, 4),
                'tz':             0.0,
                'roll':           0.0,
                'pitch':          0.0,
                'yaw':            round(yaw_cmd, 4),
                'segment_length': round(seg_len, 4),
                'heading_deg':    round(math.degrees(target_yaw), 1),
            })

        return commands

    @staticmethod
    def _angle_diff(target, current):
        """Smallest signed angle from current to target."""
        d = target - current
        while d >  math.pi: d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        return d

    # ──────────────────────────────────────────────────────────────────
    # Control loop — 20 Hz
    # ──────────────────────────────────────────────────────────────────

    def _control_loop(self):
        if self._state == 'IDLE' or self._state == 'DONE':
            return
        if not self._commands:
            return

        seg = self._commands[self._cmd_idx]
        self._seg_elapsed += self._dt

        # Publish current segment's body_pose
        twist = Twist()
        twist.linear.x  = float(seg['tx'])
        twist.linear.y  = float(seg['ty'])
        twist.linear.z  = float(seg['tz'])
        twist.angular.x = float(seg['roll'])
        twist.angular.y = float(seg['pitch'])
        twist.angular.z = float(seg['yaw'])
        self.body_pub.publish(twist)

        # Update simulated robot position
        if self._seg_elapsed <= self._seg_duration:
            self._robot_heading += seg['yaw'] / max(1, self._seg_duration / self._dt)
            self._robot_x += WALK_VEL_MS * self._dt * math.cos(self._robot_heading)
            self._robot_y += WALK_VEL_MS * self._dt * math.sin(self._robot_heading)

        # Check if segment time is up
        if self._seg_elapsed >= self._seg_duration:
            self._cmd_idx += 1

            if self._cmd_idx >= len(self._commands):
                # Path complete
                self._state = 'DONE'
                cmd_msg = String()
                cmd_msg.data = 'stop'
                self.cmd_pub.publish(cmd_msg)
                self.body_pub.publish(Twist())  # zero pose
                self.get_logger().info(
                    f'■ DONE — path complete at '
                    f'({self._robot_x:.2f}, {self._robot_y:.2f})')
                return

            # Advance to next segment
            self._seg_elapsed  = 0.0
            self._seg_duration = self._duration_for(self._commands[self._cmd_idx])

            if self._cmd_idx % 5 == 0:
                self.get_logger().info(
                    f'  Segment {self._cmd_idx}/{len(self._commands)}')

    def _duration_for(self, seg):
        """Time budget for one segment (seconds)."""
        length = float(seg.get('segment_length', 0.0))
        if length <= 0.0:
            return self._dt
        return max(self._dt, length / WALK_VEL_MS)

    # ──────────────────────────────────────────────────────────────────
    # RViz path visualization
    # ──────────────────────────────────────────────────────────────────

    def _publish_path_viz(self, world_path):
        """Publish nav_msgs/Path for RViz."""
        msg = Path()
        msg.header.frame_id = 'world'
        msg.header.stamp    = self.get_clock().now().to_msg()

        for x, y in world_path:
            ps = PoseStamped()
            ps.header.frame_id  = 'world'
            ps.pose.position.x  = float(x)
            ps.pose.position.y  = float(y)
            ps.pose.position.z  = 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)

        self.path_pub.publish(msg)


# ── Entry point ──────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LidarPathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
