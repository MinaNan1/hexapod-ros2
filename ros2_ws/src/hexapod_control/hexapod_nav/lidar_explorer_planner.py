"""
lidar_explorer_planner.py
=========================
ROS2 node — autonomous frontier-based exploration using live LiDAR data,
followed by A* path planning to a user-specified goal.

Architecture
------------
  lidar_mapper (/map)  ──►  this node  ──►  /hexapod/body_pose
                                       ──►  /hexapod/cmd
  RViz /goal_pose      ──►             ──►  /planned_path

State machine
─────────────
  IDLE → EXPLORING → (frontiers exhausted OR /goal_pose received)
       → PLANNING → WALKING → DONE
                        ↑           ↓
                        └── new /goal_pose ──┘

Subscribes:
    /map        (nav_msgs/OccupancyGrid)   — live 2D map from lidar_mapper
    /goal_pose  (geometry_msgs/PoseStamped) — goal from RViz "2D Goal Pose"

Publishes:
    /hexapod/body_pose  (geometry_msgs/Twist)  — motion commands
    /hexapod/cmd        (std_msgs/String)      — "walk" / "stop"
    /planned_path       (nav_msgs/Path)        — RViz visualization

Usage:
    ros2 run hexapod_control lidar_explorer
    ros2 launch hexapod_gazebo exploration.launch.py
"""

import math
import os
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String, Bool

import tf2_ros

# ── Import path planning modules from hexapod_path_planning_final ────
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
    from map_creation.map_inflator import MapInflator
    from map_creation.frontier_explorer import FrontierExplorer
    from path_finding.astar_planner import AStarPlanner
    from path_finding.path_smoother import PathSmoother
    _PLANNING_AVAILABLE = True
except ImportError:
    _PLANNING_AVAILABLE = False

# ── Walk speed — must match hexapod_controller.py ────────────────────
WALK_VEL_MS = 0.059     # m/s
CONTROL_HZ  = 20.0      # Hz


class LidarExplorerPlanner(Node):

    def __init__(self):
        super().__init__('lidar_explorer_planner')

        if not _PLANNING_AVAILABLE:
            self.get_logger().error(
                'Could not import path planning modules! '
                'Make sure hexapod_path_planning_final/ exists.')
            raise SystemExit(1)

        # ── ROS2 Parameters ──────────────────────────────────────────
        self.declare_parameter('robot_radius_cells',    2)
        self.declare_parameter('rdp_epsilon',           2.0)
        self.declare_parameter('waypoint_spacing',      5.0)
        self.declare_parameter('step_speed_mm',        10.0)
        self.declare_parameter('max_yaw_rad',           0.3)
        self.declare_parameter('sensor_range_cells',   15)
        self.declare_parameter('max_exploration_steps', 100)
        self.declare_parameter('auto_explore',          True)
        self.declare_parameter('explore_delay_sec',     3.0)
        # Map source — default is RTAB-Map's latched 2D grid (the production
        # SLAM stack). The old standalone pipeline used '/map' from lidar_mapper;
        # RTAB-Map replaces both that mapper AND scan_odom (better pose via loop
        # closure). RTAB-Map publishes /rtabmap/map with TRANSIENT_LOCAL QoS.
        self.declare_parameter('map_topic',             '/rtabmap/map')
        # Behaviour: 'frontier' = drive to unmapped boundaries until the space
        # is mapped (good for "what's in this room?"). 'coverage' = sweep a
        # boustrophedon (lawnmower) pattern so the camera passes over the whole
        # known floor area (the "see every part of the area" goal). Coverage
        # reuses the same A* planner + path follower, one waypoint at a time.
        self.declare_parameter('behavior',              'frontier')
        # Spacing between coverage sweep lines, in grid cells. ~ the camera's
        # useful side-coverage so adjacent passes overlap a little.
        self.declare_parameter('coverage_spacing_cells', 12)

        self.robot_radius    = self.get_parameter('robot_radius_cells').value
        self.rdp_eps         = self.get_parameter('rdp_epsilon').value
        self.wp_spacing      = self.get_parameter('waypoint_spacing').value
        self.step_speed      = self.get_parameter('step_speed_mm').value
        self.max_yaw         = self.get_parameter('max_yaw_rad').value
        self.sensor_range    = self.get_parameter('sensor_range_cells').value
        self.max_explore     = self.get_parameter('max_exploration_steps').value
        self.auto_explore    = self.get_parameter('auto_explore').value
        self.explore_delay   = self.get_parameter('explore_delay_sec').value
        self.map_topic       = self.get_parameter('map_topic').value
        self.behavior        = self.get_parameter('behavior').value
        self.coverage_spacing = int(self.get_parameter('coverage_spacing_cells').value)
        # Coverage waypoint queue (grid cells), generated lazily from the map.
        self._coverage_wps   = []
        self._coverage_built = False

        # ── Planning helpers ─────────────────────────────────────────
        self.inflator = MapInflator(robot_radius_cells=self.robot_radius)
        self.smoother = PathSmoother()

        # ── TF2 listener — read real robot position from SLAM ────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Map state ────────────────────────────────────────────────
        self.current_grid    = None     # numpy 2D (0=free, 1=obstacle)
        self.known_map       = None     # -1=unknown, 0=free, 1=obstacle
        self.map_resolution  = 0.05
        self.map_origin_x    = -5.0
        self.map_origin_y    = -5.0
        self.map_width       = 200
        self.map_height      = 200
        self._map_received   = False
        self._map_version    = 0

        # ── Exploration state ────────────────────────────────────────
        self._explore_step   = 0
        self._frontier_goal  = None
        self._explorer       = None

        # ── Path following state ─────────────────────────────────────
        self._commands       = []
        self._cmd_idx        = 0
        self._seg_elapsed    = 0.0
        self._seg_duration   = 0.0
        self._user_goal_grid = None
        self._current_path_cells = []
        self._last_checked_map_version = -1

        # ── State machine ────────────────────────────────────────────
        # WAIT_MAP | EXPLORING | PLANNING | WALKING | WALK_TO_GOAL | DONE | IDLE
        self._state          = 'WAIT_MAP'
        self._startup_timer  = 0.0

        # ── ROS2 Publishers ──────────────────────────────────────────
        self.body_pub = self.create_publisher(Twist,  '/hexapod/body_pose', 10)
        self.cmd_pub  = self.create_publisher(String, '/hexapod/cmd',       10)
        self.path_pub = self.create_publisher(Path,   '/planned_path',     10)

        # ── ROS2 Subscribers ─────────────────────────────────────────
        # RTAB-Map latches /rtabmap/map with TRANSIENT_LOCAL + RELIABLE. The
        # subscriber QoS must match or the latched map is never delivered.
        map_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            OccupancyGrid, self.map_topic, self._on_map, map_qos)
        self.create_subscription(
            PoseStamped, '/goal_pose', self._on_goal, 10)

        # ── SAFETY GATING ────────────────────────────────────────────
        # Autonomy must yield instantly to the human. Authoritative kill switch
        # is /ui/mode (Bool): True = AUTO = autonomy allowed; False = MANUAL =
        # halt immediately. We never publish /ui/mode ourselves, so there's no
        # echo/self-trigger problem. The web UI STOP button also flips to
        # MANUAL, so STOP = instant autonomy kill. Start DISABLED so merely
        # launching the node never makes a freshly-booted robot wander — the
        # operator must explicitly switch to AUTO.
        self._enabled = False
        self._was_enabled = False
        self.create_subscription(Bool, '/ui/mode', self._on_mode, 10)

        # ── Control loop — 20 Hz ─────────────────────────────────────
        self._dt = 1.0 / CONTROL_HZ
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            'LidarExplorerPlanner ready — waiting for /map')

    # ──────────────────────────────────────────────────────────────────
    # Map callback
    # ──────────────────────────────────────────────────────────────────

    def _on_map(self, msg: OccupancyGrid):
        """Convert OccupancyGrid to numpy arrays for planning."""
        self.map_resolution = msg.info.resolution
        self.map_width      = msg.info.width
        self.map_height     = msg.info.height
        self.map_origin_x   = msg.info.origin.position.x
        self.map_origin_y   = msg.info.origin.position.y

        raw = np.array(msg.data, dtype=np.int8).reshape(
            (self.map_height, self.map_width))

        # Binary grid for A*: 0=free, 1=obstacle
        grid = np.zeros_like(raw, dtype=np.int8)
        grid[raw == -1] = 1     # unknown → obstacle for navigation
        grid[raw > 30]  = 1     # occupied → obstacle

        self.current_grid = grid

        # Known map for frontier detection: -1=unknown, 0=free, 1=obstacle
        known = np.full_like(raw, -1, dtype=int)
        known[raw == 0]  = 0    # free
        known[raw > 30]  = 1    # occupied
        # Leave cells with raw == -1 as -1 (unknown)
        self.known_map = known

        self._map_version += 1
        self._map_received = True

    # ──────────────────────────────────────────────────────────────────
    # Goal callback
    # ──────────────────────────────────────────────────────────────────

    def _on_goal(self, msg: PoseStamped):
        """Receive goal from RViz — switch to goal planning mode."""
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.get_logger().info(f'Goal received: world({gx:.2f}, {gy:.2f})')

        if not self._map_received:
            self.get_logger().error('No map received yet — cannot plan.')
            return

        goal_col = int((gx - self.map_origin_x) / self.map_resolution)
        goal_row = int((gy - self.map_origin_y) / self.map_resolution)

        if not (0 <= goal_row < self.map_height and
                0 <= goal_col < self.map_width):
            self.get_logger().error(f'Goal ({goal_row}, {goal_col}) out of bounds.')
            return

        self._user_goal_grid = (goal_row, goal_col)

        # If currently exploring, stop and plan to user goal
        if self._state in ('EXPLORING', 'WALKING'):
            self._stop_walking()

        self._state = 'PLANNING'

    # ──────────────────────────────────────────────────────────────────
    # Safety: AUTO/MANUAL gate
    # ──────────────────────────────────────────────────────────────────

    def _on_mode(self, msg: Bool):
        """/ui/mode semantics (matching ui_teleop.py): data=True means MANUAL,
        data=False means AUTO. So autonomy is enabled only in AUTO mode."""
        self._enabled = (not bool(msg.data))

    # ──────────────────────────────────────────────────────────────────
    # Main tick — 20 Hz state machine
    # ──────────────────────────────────────────────────────────────────

    def _tick(self):
        # ── Safety gate ──────────────────────────────────────────────
        # When autonomy is disabled (MANUAL mode / not yet enabled), make sure
        # the robot is stopped and do nothing else. On the enabled→disabled
        # edge, send one stop so any in-progress motion halts immediately.
        if not self._enabled:
            if self._was_enabled:
                self._stop_walking()
                self.get_logger().info('Autonomy DISABLED (MANUAL) — halted.')
                self._was_enabled = False
            return
        if not self._was_enabled:
            self.get_logger().info('Autonomy ENABLED (AUTO).')
            self._was_enabled = True

        if self._state == 'WAIT_MAP':
            if self._map_received:
                self._startup_timer += self._dt
                if self._startup_timer >= self.explore_delay:
                    if self.auto_explore:
                        self.get_logger().info('Map received — starting exploration')
                        self._state = 'EXPLORING'
                    else:
                        self.get_logger().info('Map received — waiting for /goal_pose')
                        self._state = 'IDLE'
            return

        if self._state == 'IDLE':
            return

        if self._state == 'EXPLORING':
            if self.behavior == 'coverage':
                self._tick_coverage()
            else:
                self._tick_explore()
            return

        if self._state == 'PLANNING':
            self._plan_to_user_goal()
            return

        if self._state in ('WALKING', 'WALK_TO_GOAL'):
            self._tick_walk()
            return

        if self._state == 'DONE':
            return

    # ──────────────────────────────────────────────────────────────────
    # Exploration logic
    # ──────────────────────────────────────────────────────────────────

    def _tick_explore(self):
        """One exploration step: find frontier, plan, walk."""
        if self._explore_step >= self.max_explore:
            self.get_logger().info(
                f'Exploration limit reached ({self.max_explore} steps)')
            self._state = 'IDLE'
            return

        if self.known_map is None:
            return

        # Create a fresh FrontierExplorer each cycle (map may have changed)
        inflated = self._safe_inflate(self.current_grid)
        explorer = FrontierExplorer(
            self.known_map, inflated, self.robot_radius)

        frontier = explorer.find_frontier()
        self._explore_step += 1

        if len(frontier) == 0:
            self.get_logger().info(
                f'No more frontier — exploration complete after '
                f'{self._explore_step} steps')
            self._state = 'IDLE'
            return

        # Filter frontiers that are too close (already in sensor range)
        robot_row, robot_col = self._robot_grid_pos()
        useful = [
            f for f in frontier
            if abs(f[0] - robot_row) + abs(f[1] - robot_col)
            > self.sensor_range
        ] or frontier

        goal = explorer.pick_nearest_frontier(
            (robot_row, robot_col), useful)

        if goal is None:
            self.get_logger().info('No reachable frontier goal')
            self._state = 'IDLE'
            return

        self.get_logger().info(
            f'Explore step {self._explore_step}: '
            f'{len(frontier)} frontier cells, goal={goal}')

        # Plan A* to frontier goal
        success = self._plan_path_to(goal, mode='WALKING')
        if not success:
            self.get_logger().warn(
                f'Cannot reach frontier {goal} — trying next step')
            # Don't stop — next tick will pick a different frontier

    # ──────────────────────────────────────────────────────────────────
    # Coverage logic (boustrophedon / lawnmower sweep)
    # ──────────────────────────────────────────────────────────────────

    def _tick_coverage(self):
        """Drive a boustrophedon sweep so the camera passes over the whole
        known floor. Reuses the A* planner + path follower one waypoint at a
        time; WALKING completion returns here for the next waypoint."""
        if self.known_map is None:
            return

        # Build the sweep once we have a map; rebuild if we ran the queue dry
        # (newly-revealed free space gets a fresh sweep).
        if not self._coverage_wps:
            self._coverage_wps = self._generate_boustrophedon()
            if not self._coverage_wps:
                self.get_logger().info('Coverage: no reachable free area — done.')
                self._state = 'IDLE'
                return
            self.get_logger().info(
                f'Coverage: {len(self._coverage_wps)} waypoints planned.')

        inflated = self._safe_inflate(self.current_grid)
        robot_row, robot_col = self._robot_grid_pos()

        # Pop the next waypoint that is still free and not basically underfoot.
        while self._coverage_wps:
            wp = self._coverage_wps.pop(0)
            r, c = wp
            if not (0 <= r < self.map_height and 0 <= c < self.map_width):
                continue
            if inflated[r, c] == 1:                       # became an obstacle
                continue
            if abs(r - robot_row) + abs(c - robot_col) < self.coverage_spacing // 2:
                continue                                  # already here
            if self._plan_path_to(wp, mode='WALKING'):
                return                                    # walking to it now
            # else: unreachable — try the next waypoint
        # Queue exhausted → loop will rebuild next tick (or go IDLE if empty).

    def _generate_boustrophedon(self):
        """Return an ordered list of (row, col) grid waypoints covering the
        known free space in alternating left/right sweep lines."""
        import numpy as _np
        free = (self.known_map == 0)
        inflated = self._safe_inflate(self.current_grid)
        free &= (inflated == 0)                           # keep a robot-radius margin
        rows = _np.where(free.any(axis=1))[0]
        if rows.size == 0:
            return []
        r0, r1 = int(rows.min()), int(rows.max())

        wps = []
        flip = False
        for r in range(r0, r1 + 1, max(1, self.coverage_spacing)):
            cols = _np.where(free[r])[0]
            if cols.size == 0:
                continue
            c_lo, c_hi = int(cols.min()), int(cols.max())
            line = list(range(c_lo, c_hi + 1, max(1, self.coverage_spacing)))
            if flip:
                line.reverse()
            wps.extend((r, c) for c in line)
            flip = not flip
        return wps

    # ──────────────────────────────────────────────────────────────────
    # Path planning
    # ──────────────────────────────────────────────────────────────────

    def _plan_to_user_goal(self):
        """Plan A* path to user-specified goal."""
        if self._user_goal_grid is None:
            self._state = 'IDLE'
            return

        self.get_logger().info(
            f'Planning to user goal {self._user_goal_grid}')

        success = self._plan_path_to(
            self._user_goal_grid, mode='WALK_TO_GOAL')

        if not success:
            self.get_logger().error('No path to user goal')
            self._state = 'IDLE'
        else:
            self._user_goal_grid = None

    def _plan_path_to(self, goal_grid, mode='WALKING'):
        """Run A* from robot position to goal, prepare walk commands."""
        if self.current_grid is None:
            return False

        start = self._robot_grid_pos()

        # Clamp to bounds
        start = (
            max(0, min(start[0], self.map_height - 1)),
            max(0, min(start[1], self.map_width  - 1)))

        inflated = self._safe_inflate(self.current_grid)

        # Clear area around robot if it's inside an obstacle
        if inflated[start[0], start[1]] == 1:
            self.get_logger().warn('Start is in obstacle — clearing local area')
            r, c = start
            for dr in range(-3, 4):
                for dc in range(-3, 4):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < self.map_height and 0 <= nc < self.map_width:
                        inflated[nr, nc] = 0

        if inflated[goal_grid[0], goal_grid[1]] == 1:
            self.get_logger().warn(f'Goal {goal_grid} is in obstacle')
            return False

        planner = AStarPlanner(inflated, start, goal_grid)
        result  = planner.plan()

        if not result.found:
            self.get_logger().warn(
                f'A* failed: {start}→{goal_grid} '
                f'(explored {result.explored_count} cells)')
            return False

        # Smooth path
        rdp_path    = self.smoother.rdp_simplify(
            result.path, epsilon=self.rdp_eps)
        smooth_path = self.smoother.interpolate(
            rdp_path, spacing=self.wp_spacing)

        self._current_path_cells = smooth_path
        self._last_checked_map_version = self._map_version

        self.get_logger().info(
            f'Path: {len(result.path)} raw → {len(smooth_path)} smooth '
            f'(cost={result.cost:.1f})')

        # Convert to world coordinates
        world_path = [
            (self.map_origin_x + col * self.map_resolution,
             self.map_origin_y + row * self.map_resolution)
            for row, col in smooth_path
        ]

        # Generate body-pose commands
        self._commands = self._path_to_commands(world_path)
        self._cmd_idx       = 0
        self._seg_elapsed   = 0.0

        # Publish path for RViz
        self._publish_path_viz(world_path)

        # Start walking
        if self._commands:
            self._state = mode
            self._seg_duration = self._duration_for(self._commands[0])
            cmd_msg = String()
            cmd_msg.data = 'walk'
            self.cmd_pub.publish(cmd_msg)
            self.get_logger().info(
                f'▶ WALKING — {len(self._commands)} segments')

        return bool(self._commands)

    # ──────────────────────────────────────────────────────────────────
    # Walk execution
    # ──────────────────────────────────────────────────────────────────

    def _tick_walk(self):
        """Execute one tick of path following."""
        if not self._commands:
            return

        # --- Dynamic Replanning Check ---
        if self._map_version > self._last_checked_map_version:
            self._last_checked_map_version = self._map_version
            inflated = self._safe_inflate(self.current_grid)
            
            collision = False
            for i in range(self._cmd_idx, len(self._current_path_cells)):
                r, c = self._current_path_cells[i]
                if 0 <= r < self.map_height and 0 <= c < self.map_width:
                    if inflated[r, c] == 1:
                        collision = True
                        break
            
            if collision:
                self.get_logger().warn('Obstacle detected on upcoming path! Stopping to replan...')
                prev_state = self._state
                self._stop_walking()
                
                if prev_state == 'WALKING':
                    self._state = 'EXPLORING'
                elif prev_state == 'WALK_TO_GOAL':
                    self._state = 'PLANNING'
                return
        # --------------------------------

        seg = self._commands[self._cmd_idx]
        self._seg_elapsed += self._dt

        # Publish body_pose
        twist = Twist()
        twist.linear.x  = float(seg['tx'])
        twist.linear.y  = float(seg['ty'])
        twist.linear.z  = float(seg['tz'])
        twist.angular.x = float(seg['roll'])
        twist.angular.y = float(seg['pitch'])
        twist.angular.z = float(seg['yaw'])
        self.body_pub.publish(twist)

        # Position is now tracked via TF (SLAM), no dead-reckoning needed

        # Segment complete?
        if self._seg_elapsed >= self._seg_duration:
            self._cmd_idx += 1

            if self._cmd_idx >= len(self._commands):
                self._stop_walking()
                if self._state == 'WALK_TO_GOAL':
                    pose = self._get_robot_pose()
                    pos_str = f'({pose[0]:.2f}, {pose[1]:.2f})' if pose else '(unknown)'
                    self.get_logger().info(
                        f'■ DONE — reached goal at {pos_str}')
                    self._state = 'DONE'
                else:
                    # Was exploring — go back to exploring
                    self._state = 'EXPLORING'
                return

            self._seg_elapsed  = 0.0
            self._seg_duration = self._duration_for(
                self._commands[self._cmd_idx])

            if self._cmd_idx % 5 == 0:
                self.get_logger().info(
                    f'  Segment {self._cmd_idx}/{len(self._commands)}')

    def _stop_walking(self):
        """Stop the hexapod."""
        cmd_msg = String()
        cmd_msg.data = 'stop'
        self.cmd_pub.publish(cmd_msg)
        self.body_pub.publish(Twist())
        self._commands = []
        self._cmd_idx  = 0
        self._current_path_cells = []

    # ──────────────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────────────

    def _get_robot_pose(self):
        """Read current robot pose from TF (map → base_link).

        Returns (x, y, yaw) in world metres, or None if TF unavailable.
        """
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            # Quaternion to yaw (inline — no tf_transformations dependency)
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (x, y, yaw)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return None

    def _robot_grid_pos(self):
        """Current robot position in grid coords (from SLAM TF)."""
        pose = self._get_robot_pose()
        if pose is None:
            # Fallback: center of map
            return (self.map_height // 2, self.map_width // 2)
        x, y, _ = pose
        col = int((x - self.map_origin_x) / self.map_resolution)
        row = int((y - self.map_origin_y) / self.map_resolution)
        col = max(0, min(col, self.map_width  - 1))
        row = max(0, min(row, self.map_height - 1))
        return (row, col)

    def _safe_inflate(self, grid):
        """Inflate obstacles, catching any errors."""
        try:
            return self.inflator.inflate(grid)
        except Exception as e:
            self.get_logger().error(f'Inflation failed: {e}')
            return grid.copy()

    def _path_to_commands(self, world_path):
        """Convert world-coordinate path to hexapod body_pose commands."""
        if len(world_path) < 2:
            return []

        commands = []
        pose = self._get_robot_pose()
        current_yaw = pose[2] if pose else 0.0

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
                tx = ty = 0.0

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

    def _duration_for(self, seg):
        """Time budget for one segment (seconds)."""
        length = float(seg.get('segment_length', 0.0))
        if length <= 0.0:
            return self._dt
        return max(self._dt, length / WALK_VEL_MS)

    def _publish_path_viz(self, world_path):
        """Publish nav_msgs/Path for RViz."""
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()

        for x, y in world_path:
            ps = PoseStamped()
            ps.header.frame_id  = 'map'
            ps.pose.position.x  = float(x)
            ps.pose.position.y  = float(y)
            ps.pose.position.z  = 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)

        self.path_pub.publish(msg)


# ── Entry point ──────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LidarExplorerPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
