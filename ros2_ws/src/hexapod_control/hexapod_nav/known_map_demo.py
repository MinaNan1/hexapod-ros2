"""
known_map_demo.py
=================
Approach 1 demo — known map + A* path planning, no LiDAR.

What it does
------------
1. Generates a random 2D occupancy grid (MapGenerator).
2. Start = world (0, 0) — where hexapod_controller starts integrating.
3. Picks a random reachable goal in free space.
4. Inflates obstacles, runs A*, smooths the path.
5. Publishes the map and the path so RViz can show them.
6. Drives the hexapod by closed-loop steering:
     - The hexapod_controller has constant forward speed.  Only the
       `yaw` field of /hexapod/body_pose actually changes its heading.
     - Each tick, look up base_link in odom via TF, find the next
       waypoint, send a yaw command proportional to heading error.
     - When close to the goal, publish "stop" on /hexapod/cmd.

Topics
------
    /map                nav_msgs/OccupancyGrid
    /planned_path       nav_msgs/Path
    /hexapod/body_pose  geometry_msgs/Twist     (only angular.z used)
    /hexapod/cmd        std_msgs/String         ("walk" / "stop")

Usage
-----
    # Terminal 1 — controller
    source ~/hexapod-ros2/hexapod_ws/install/setup.bash
    ros2 run hexapod_control hexapod_controller

    # Terminal 2 — this demo
    cd ~/hexapod-ros2/ros2_ws && source install/setup.bash
    ros2 run hexapod_control known_map_demo

    # Terminal 3 — RViz, Fixed Frame: odom
    rviz2

Add in RViz:
    Map  on /map
    Path on /planned_path
    TF
"""

import math
import os
import sys

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, TransformException


# ── Make the path-planning library importable ────────────────────────
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
    return candidates[2]  # return container path for error message

_PLANNING_PATH = _find_planning_path()
sys.path.insert(0, _PLANNING_PATH)

try:
    from map_creation.map_generator import MapGenerator
    from map_creation.map_inflator   import MapInflator
    from path_finding.astar_planner  import AStarPlanner
    from path_finding.path_smoother  import PathSmoother
    _PLANNING_AVAILABLE = True
except ImportError:
    _PLANNING_AVAILABLE = False


CONTROL_HZ = 20.0


class KnownMapDemo(Node):

    def __init__(self):
        super().__init__('known_map_demo')

        if not _PLANNING_AVAILABLE:
            self.get_logger().fatal(
                'Path-planning modules not found. Expected at '
                f'{_PLANNING_PATH}')
            raise SystemExit(1)

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('grid_width',         200)
        self.declare_parameter('grid_height',        200)
        self.declare_parameter('resolution',         0.05)   # m/cell
        self.declare_parameter('map_type',           'random')
        self.declare_parameter('seed',               -1)     # -1 → random
        # robot_radius_cells must cover the full leg span, not just the body.
        # Stance foot tip is ~34 cm from body centre → 7 cells @ 0.05 m/cell.
        # Default leaves an extra 4 cells buffer so legs stay clear during
        # turns, swings, and any small heading drift.
        self.declare_parameter('robot_radius_cells', 8)    # full leg footprint
        self.declare_parameter('safety_margin_cells', 4)   # extra wall buffer
        self.declare_parameter('rdp_epsilon',        2.0)
        self.declare_parameter('waypoint_spacing',   3.0)
        self.declare_parameter('min_goal_dist_m',    2.0)
        self.declare_parameter('wall_height_m',      0.30)  # 3D extrusion height
        self.declare_parameter('frame_id',           'odom')
        self.declare_parameter('robot_frame',        'base_link')
        self.declare_parameter('startup_delay_sec',  3.0)
        self.declare_parameter('plan_attempts',      10)
        # Steering control
        self.declare_parameter('lookahead_m',        0.30)
        self.declare_parameter('goal_tolerance_m',   0.20)
        self.declare_parameter('yaw_kp',             2.0)
        self.declare_parameter('max_yaw_rate',       0.6)    # rad/s

        self.grid_w   = self.get_parameter('grid_width').value
        self.grid_h   = self.get_parameter('grid_height').value
        self.res      = self.get_parameter('resolution').value
        map_type      = self.get_parameter('map_type').value
        seed          = self.get_parameter('seed').value
        self.r_cells     = self.get_parameter('robot_radius_cells').value
        self.safety_cells = self.get_parameter('safety_margin_cells').value
        self.rdp_eps     = self.get_parameter('rdp_epsilon').value
        self.wp_step     = self.get_parameter('waypoint_spacing').value
        self.min_dist    = self.get_parameter('min_goal_dist_m').value
        self.wall_height = float(self.get_parameter('wall_height_m').value)
        self.frame       = self.get_parameter('frame_id').value
        self.robot_frame = self.get_parameter('robot_frame').value
        startup_delay = float(self.get_parameter('startup_delay_sec').value)
        attempts      = self.get_parameter('plan_attempts').value
        self.lookahead = float(self.get_parameter('lookahead_m').value)
        self.goal_tol  = float(self.get_parameter('goal_tolerance_m').value)
        self.yaw_kp    = float(self.get_parameter('yaw_kp').value)
        self.max_yaw   = float(self.get_parameter('max_yaw_rate').value)

        self.origin_x = -self.grid_w * self.res / 2.0
        self.origin_y = -self.grid_h * self.res / 2.0

        # ── Publishers up FIRST so /map and /planned_path appear in
        # `ros2 topic list` even if planning later fails. ────────────
        self.map_pub   = self.create_publisher(OccupancyGrid, '/map',           10)
        self.path_pub  = self.create_publisher(Path,          '/planned_path',  10)
        self.walls_pub = self.create_publisher(Marker,        '/walls_3d',      10)
        self.body_pub  = self.create_publisher(Twist,         '/hexapod/body_pose', 10)
        self.cmd_pub   = self.create_publisher(String,        '/hexapod/cmd',   10)
        self._walls_msg = None

        # ── Freeze the robot immediately so it doesn't wander off
        # while we plan.  hexapod_controller defaults to walking=True. ─
        self._send_cmd('stop')

        # ── TF listener for closed-loop steering ─────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Plan (with retries) ──────────────────────────────────────
        if seed < 0:
            seed = int(np.random.randint(0, 1_000_000))

        self.raw_map    = None
        self.world_path = None
        for attempt in range(attempts):
            ok = self._try_plan(map_type, seed + attempt)
            if ok:
                break
            self.get_logger().warn(
                f'Plan attempt {attempt + 1}/{attempts} failed, retrying...')
        if self.world_path is None:
            self.get_logger().fatal(
                f'All {attempts} plan attempts failed.  '
                f'Try a different map_type or seed.')
            raise SystemExit(1)

        # ── Map + path republished at 1 Hz so RViz keeps them alive ──
        self._publish_map_and_path()
        self.create_timer(1.0, self._publish_map_and_path)

        # ── Walk loop @ 20 Hz ────────────────────────────────────────
        self._dt            = 1.0 / CONTROL_HZ
        self._state         = 'WAIT'
        self._wait_elapsed  = 0.0
        self._startup_delay = startup_delay
        self._wp_idx        = 0
        self._tf_warned     = False
        self.create_timer(self._dt, self._walk_tick)

        self.get_logger().info(
            f'▶ Path planned, robot will walk in {startup_delay:.1f} s')

    # ──────────────────────────────────────────────────────────────────
    # Planning (with retries)
    # ──────────────────────────────────────────────────────────────────

    def _try_plan(self, map_type, seed):
        """Generate a map + goal and run A*.  Returns True on success."""
        rng = np.random.RandomState(seed)
        self.get_logger().info(
            f'Generating "{map_type}" map (seed={seed}) '
            f'{self.grid_w}×{self.grid_h} @ {self.res} m/cell')

        gen = MapGenerator(width=self.grid_w, height=self.grid_h)
        if map_type == 'walls':
            raw = gen.walls()
        elif map_type == 'maze':
            raw = gen.maze()
        elif map_type == 'shapes':
            raw = gen.shapes()
        else:
            raw = gen.random_obstacles(num_obstacles=12, seed=seed)

        # Inflation = robot body radius + extra safety margin.  The planner
        # treats every cell within this distance of an obstacle as blocked,
        # so the path keeps the robot centre that far from any wall.
        total_radius = int(self.r_cells) + int(self.safety_cells)
        inflator = MapInflator(robot_radius_cells=total_radius)
        inflated = inflator.inflate(raw)

        # Start = world (0, 0)
        start = self._world_to_grid(0.0, 0.0)
        # Stamp a small free zone around start in case the random map
        # dropped an obstacle on the spawn point.
        for dr in range(-3, 4):
            for dc in range(-3, 4):
                nr, nc = start[0] + dr, start[1] + dc
                if 0 <= nr < self.grid_h and 0 <= nc < self.grid_w:
                    inflated[nr, nc] = 0
                    raw[nr, nc]      = 0

        goal = self._pick_random_goal(start, inflated, rng)
        if goal is None:
            return False

        planner = AStarPlanner(inflated, start, goal)
        result  = planner.plan()
        if not result.found:
            return False

        smoother    = PathSmoother()
        rdp_path    = smoother.rdp_simplify(result.path, epsilon=self.rdp_eps)
        smooth_path = smoother.interpolate(rdp_path, spacing=self.wp_step)

        # Publish-ready data
        self.raw_map    = raw
        self.world_path = [self._grid_to_world(wp) for wp in smooth_path]
        self._walls_msg = self._build_walls_marker(raw)

        gx, gy = self._grid_to_world(goal)
        self.get_logger().info(
            f'  Goal:  world ({gx:+.2f}, {gy:+.2f}) m')
        self.get_logger().info(
            f'  A* path: {len(result.path)} raw → '
            f'{len(smooth_path)} smooth waypoints (cost {result.cost:.1f})')
        self.get_logger().info(
            f'  Inflation: robot {self.r_cells} + safety {self.safety_cells} '
            f'= {total_radius} cells '
            f'({total_radius * self.res * 100:.0f} cm clearance)')
        return True

    # ──────────────────────────────────────────────────────────────────
    # 3D walls — extrude every obstacle cell into a coloured cube
    # ──────────────────────────────────────────────────────────────────

    def _build_walls_marker(self, raw_map):
        """Return a single Marker (CUBE_LIST) with one cube per obstacle cell."""
        m = Marker()
        m.header.frame_id = self.frame
        m.ns              = 'walls'
        m.id              = 0
        m.type            = Marker.CUBE_LIST
        m.action          = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = float(self.res)
        m.scale.y = float(self.res)
        m.scale.z = float(self.wall_height)
        m.color.r = 0.30
        m.color.g = 0.32
        m.color.b = 0.40
        m.color.a = 0.95

        obstacles = np.argwhere(raw_map == 1)
        z_centre  = self.wall_height / 2.0
        for r, c in obstacles:
            p = Point()
            p.x = float(self.origin_x + (c + 0.5) * self.res)
            p.y = float(self.origin_y + (r + 0.5) * self.res)
            p.z = float(z_centre)
            m.points.append(p)

        return m

    def _pick_random_goal(self, start, inflated, rng):
        free = np.argwhere(inflated == 0)
        if len(free) == 0:
            return None
        min_cells = self.min_dist / self.res
        for _ in range(300):
            idx = rng.randint(0, len(free))
            r, c = free[idx]
            if math.hypot(r - start[0], c - start[1]) >= min_cells:
                return (int(r), int(c))
        return None

    # ──────────────────────────────────────────────────────────────────
    # Coordinate helpers
    # ──────────────────────────────────────────────────────────────────

    def _world_to_grid(self, x, y):
        col = int((x - self.origin_x) / self.res)
        row = int((y - self.origin_y) / self.res)
        col = max(0, min(col, self.grid_w - 1))
        row = max(0, min(row, self.grid_h - 1))
        return (row, col)

    def _grid_to_world(self, rc):
        # Cell centre — matches the OccupancyGrid convention and the wall
        # cube positions, so the path doesn't visually clip into walls.
        row, col = rc
        return (self.origin_x + (col + 0.5) * self.res,
                self.origin_y + (row + 0.5) * self.res)

    # ──────────────────────────────────────────────────────────────────
    # Closed-loop steering
    # ──────────────────────────────────────────────────────────────────

    def _walk_tick(self):
        if self._state == 'WAIT':
            self._wait_elapsed += self._dt
            if self._wait_elapsed >= self._startup_delay:
                self._send_cmd('walk')
                self._state = 'WALKING'
                self.get_logger().info(
                    f'▶ WALKING — tracking {len(self.world_path)} waypoints')
            return

        if self._state == 'DONE':
            return

        # Get robot pose
        pose = self._lookup_robot_pose()
        if pose is None:
            return
        rx, ry, rh = pose

        # Goal check (Euclidean to last waypoint)
        gx, gy = self.world_path[-1]
        dist_to_goal = math.hypot(gx - rx, gy - ry)
        if dist_to_goal < self.goal_tol:
            self._finish()
            return

        # Advance the lookahead waypoint pointer
        while self._wp_idx < len(self.world_path) - 1:
            wx, wy = self.world_path[self._wp_idx]
            if math.hypot(wx - rx, wy - ry) < self.lookahead:
                self._wp_idx += 1
            else:
                break

        # Steer toward current target
        tx, ty = self.world_path[self._wp_idx]
        desired_heading = math.atan2(ty - ry, tx - rx)
        heading_error   = self._angle_diff(desired_heading, rh)

        yaw_cmd = max(-self.max_yaw,
                      min(self.max_yaw, self.yaw_kp * heading_error))

        twist = Twist()
        twist.angular.z = float(yaw_cmd)
        # tx/ty/tz/roll/pitch ignored by controller — leave at 0
        self.body_pub.publish(twist)

        # Periodic progress log
        if int(self._wait_elapsed * CONTROL_HZ) % 40 == 0:
            self.get_logger().info(
                f'  pose=({rx:+.2f},{ry:+.2f}, {math.degrees(rh):+.0f}°) '
                f'| wp {self._wp_idx}/{len(self.world_path) - 1} '
                f'| dist_to_goal={dist_to_goal:.2f} m '
                f'| yaw_cmd={yaw_cmd:+.2f}')
        self._wait_elapsed += self._dt   # also acts as a tick counter

    def _finish(self):
        self._state = 'DONE'
        self._send_cmd('stop')
        self.body_pub.publish(Twist())
        self.get_logger().info('■ DONE — goal reached')

    def _lookup_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame, self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except TransformException as ex:
            if not self._tf_warned:
                self._tf_warned = True
                self.get_logger().warn(
                    f'TF {self.frame} ← {self.robot_frame} not yet available: {ex}')
            return None
        if self._tf_warned:
            self._tf_warned = False
            self.get_logger().info(
                f'TF {self.frame} ← {self.robot_frame} now available')
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w
        yaw = math.atan2(2 * (qw * qz + qx * qy),
                         1 - 2 * (qy * qy + qz * qz))
        return x, y, yaw

    @staticmethod
    def _angle_diff(target, current):
        d = target - current
        while d >  math.pi: d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        return d

    # ──────────────────────────────────────────────────────────────────
    # Publishing helpers
    # ──────────────────────────────────────────────────────────────────

    def _send_cmd(self, text):
        msg = String()
        msg.data = text
        self.cmd_pub.publish(msg)

    def _publish_map_and_path(self):
        if self.raw_map is None or self.world_path is None:
            return
        stamp = self.get_clock().now().to_msg()

        og = OccupancyGrid()
        og.header.stamp        = stamp
        og.header.frame_id     = self.frame
        og.info.resolution     = float(self.res)
        og.info.width          = self.grid_w
        og.info.height         = self.grid_h
        og.info.origin.position.x    = float(self.origin_x)
        og.info.origin.position.y    = float(self.origin_y)
        og.info.origin.orientation.w = 1.0
        flat = self.raw_map.astype(np.int8).flatten()
        og.data = np.where(flat == 1, 100, 0).astype(np.int8).tolist()
        self.map_pub.publish(og)

        pmsg = Path()
        pmsg.header.stamp    = stamp
        pmsg.header.frame_id = self.frame
        for x, y in self.world_path:
            ps = PoseStamped()
            ps.header.frame_id = self.frame
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            pmsg.poses.append(ps)
        self.path_pub.publish(pmsg)

        if self._walls_msg is not None:
            self._walls_msg.header.stamp = stamp
            self.walls_pub.publish(self._walls_msg)


# ── Entry point ──────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = KnownMapDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
