"""
room_corners.py
===============
ROS2 node — given the room dimensions and which corner the hexapod is
starting at, compute the four corner positions in the world frame and
continuously report which corner is currently nearest to the robot.

Corner labels (top-down view of the room)
-----------------------------------------
        +y
         ^
    B ───┼─── A
    │    │    │
    │    └────┼─── +x
    │         │
    D ─────── C

    A = top-right   (max x, max y)
    B = top-left    (min x, max y)
    C = bottom-right(max x, min y)
    D = bottom-left (min x, min y)

The robot always starts at world origin (0, 0).  Tell the node which
corner that origin is — the other three are then placed automatically:

    start=A → room extends in (-x, -y)
    start=B → room extends in (+x, -y)
    start=C → room extends in (-x, +y)
    start=D → room extends in (+x, +y)

Topics published
----------------
    /room_corners    visualization_msgs/MarkerArray
        Four labelled spheres at A, B, C, D for RViz.

    /nearest_corner  geometry_msgs/PoseStamped
        Pose of whichever corner is currently closest to base_link.
        Updated at `publish_rate` Hz.  Use it as a navigation goal, or
        just watch it tick over.

Usage
-----
    ros2 run hexapod_control room_corners --ros-args \\
        -p room_width_m:=4.0 \\
        -p room_height_m:=3.0 \\
        -p start_corner:=A

In RViz add:
    - MarkerArray on /room_corners (fixed frame: world)
    - Pose       on /nearest_corner

The terminal logs the nearest corner whenever it changes.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException


CORNER_NAMES = ('A', 'B', 'C', 'D')

CORNER_COLORS = {
    'A': (1.00, 0.20, 0.20),  # red
    'B': (0.20, 0.85, 0.20),  # green
    'C': (0.25, 0.45, 1.00),  # blue
    'D': (1.00, 0.85, 0.20),  # yellow
}


class RoomCorners(Node):

    def __init__(self):
        super().__init__('room_corners')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('room_width_m',   3.0)   # extent along x
        self.declare_parameter('room_height_m',  3.0)   # extent along y
        self.declare_parameter('start_corner',   'A')   # A | B | C | D
        self.declare_parameter('world_frame',    'world')
        self.declare_parameter('robot_frame',    'base_link')
        self.declare_parameter('publish_rate',   2.0)   # Hz

        self.W = float(self.get_parameter('room_width_m').value)
        self.H = float(self.get_parameter('room_height_m').value)
        start  = str(self.get_parameter('start_corner').value).upper().strip()
        self.world_frame = self.get_parameter('world_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        rate   = float(self.get_parameter('publish_rate').value)

        if start not in CORNER_NAMES:
            self.get_logger().fatal(
                f'Invalid start_corner "{start}" — must be one of A, B, C, D')
            raise SystemExit(1)

        if self.W <= 0 or self.H <= 0:
            self.get_logger().fatal(
                f'Room size must be positive — got {self.W} x {self.H}')
            raise SystemExit(1)

        # ── Compute the four corner positions in world frame ─────────
        self.start_corner = start
        self.corners      = self._compute_corners(start)

        self.get_logger().info(
            f'Room {self.W:.2f} m × {self.H:.2f} m | '
            f'robot starts at corner {start}')
        for name in CORNER_NAMES:
            x, y = self.corners[name]
            tag  = ' ← start' if name == start else ''
            self.get_logger().info(
                f'  Corner {name}: ({x:+.2f}, {y:+.2f}) m{tag}')

        # ── TF for the live robot pose ────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Publishers ────────────────────────────────────────────────
        self.markers_pub = self.create_publisher(
            MarkerArray, '/room_corners', 10)
        self.nearest_pub = self.create_publisher(
            PoseStamped, '/nearest_corner', 10)

        # ── Tick ──────────────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._tick)
        self._last_nearest = None
        self._tf_warned    = False

    # ──────────────────────────────────────────────────────────────────
    # Geometry — corner positions for each possible start corner
    # ──────────────────────────────────────────────────────────────────

    def _compute_corners(self, start):
        """Return dict of {name: (x, y)} for the four corners.

        The robot is always at world (0, 0) and the room is axis-aligned
        in the world frame.  The chosen start corner sits at the origin;
        the room extends away from it according to the labelling convention
        in the module docstring.
        """
        W, H = self.W, self.H

        # Pre-tabulated layouts so the choice is unambiguous.
        layouts = {
            'A': {'A': (0.0,  0.0), 'B': (-W,  0.0),
                  'C': (0.0, -H),   'D': (-W, -H)},
            'B': {'A': (W,    0.0), 'B': (0.0, 0.0),
                  'C': (W,   -H),   'D': (0.0, -H)},
            'C': {'A': (0.0,  H),   'B': (-W,  H),
                  'C': (0.0,  0.0), 'D': (-W,  0.0)},
            'D': {'A': (W,    H),   'B': (0.0, H),
                  'C': (W,    0.0), 'D': (0.0, 0.0)},
        }
        return layouts[start]

    # ──────────────────────────────────────────────────────────────────
    # Per-tick: publish markers and the current nearest corner
    # ──────────────────────────────────────────────────────────────────

    def _tick(self):
        # 1. Always re-publish the markers so RViz keeps them fresh
        self.markers_pub.publish(self._build_markers())

        # 2. Look up the robot pose
        robot_xy = self._lookup_robot_xy()
        if robot_xy is None:
            return
        rx, ry = robot_xy

        # 3. Find the nearest corner
        nearest_name = min(
            CORNER_NAMES,
            key=lambda n: math.hypot(rx - self.corners[n][0],
                                     ry - self.corners[n][1]))
        cx, cy = self.corners[nearest_name]
        dist   = math.hypot(rx - cx, ry - cy)

        # 4. Publish as a navigation-ready PoseStamped
        msg = PoseStamped()
        msg.header.stamp        = self.get_clock().now().to_msg()
        msg.header.frame_id     = self.world_frame
        msg.pose.position.x     = float(cx)
        msg.pose.position.y     = float(cy)
        msg.pose.position.z     = 0.0
        msg.pose.orientation.w  = 1.0
        self.nearest_pub.publish(msg)

        # 5. Log only when the nearest corner changes (avoid spam)
        if self._last_nearest != nearest_name:
            self._last_nearest = nearest_name
            self.get_logger().info(
                f'Nearest corner: {nearest_name} at '
                f'({cx:+.2f}, {cy:+.2f}) m — {dist:.2f} m away '
                f'(robot at {rx:+.2f}, {ry:+.2f})')

    def _lookup_robot_xy(self):
        """Get the robot's (x, y) in world frame, or None if TF unavailable."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.robot_frame,
                rclpy.time.Time(),                # latest available
                timeout=Duration(seconds=0.1),
            )
        except TransformException as ex:
            if not self._tf_warned:
                self._tf_warned = True
                self.get_logger().warn(
                    f'Cannot get TF {self.world_frame} ← {self.robot_frame}: {ex}')
            return None

        if self._tf_warned:
            self._tf_warned = False
            self.get_logger().info(
                f'TF {self.world_frame} ← {self.robot_frame} now available')

        return tf.transform.translation.x, tf.transform.translation.y

    # ──────────────────────────────────────────────────────────────────
    # RViz markers — four spheres + four text labels
    # ──────────────────────────────────────────────────────────────────

    def _build_markers(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, name in enumerate(CORNER_NAMES):
            x, y = self.corners[name]
            r, g, b = CORNER_COLORS[name]

            # Sphere at the corner
            sphere = Marker()
            sphere.header.frame_id   = self.world_frame
            sphere.header.stamp      = stamp
            sphere.ns                = 'corners'
            sphere.id                = i
            sphere.type              = Marker.SPHERE
            sphere.action            = Marker.ADD
            sphere.pose.position.x   = float(x)
            sphere.pose.position.y   = float(y)
            sphere.pose.position.z   = 0.05
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.18
            sphere.color.r, sphere.color.g, sphere.color.b = r, g, b
            sphere.color.a = 1.0
            arr.markers.append(sphere)

            # Text label above each sphere
            label = Marker()
            label.header.frame_id    = self.world_frame
            label.header.stamp       = stamp
            label.ns                 = 'corner_labels'
            label.id                 = i
            label.type               = Marker.TEXT_VIEW_FACING
            label.action             = Marker.ADD
            label.pose.position.x    = float(x)
            label.pose.position.y    = float(y)
            label.pose.position.z    = 0.30
            label.pose.orientation.w = 1.0
            label.scale.z            = 0.22
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a            = 1.0
            label.text               = name
            arr.markers.append(label)

        # Outline: connect the four corners with a line strip
        outline = Marker()
        outline.header.frame_id    = self.world_frame
        outline.header.stamp       = stamp
        outline.ns                 = 'room_outline'
        outline.id                 = 0
        outline.type               = Marker.LINE_STRIP
        outline.action             = Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.scale.x            = 0.03
        outline.color.r, outline.color.g, outline.color.b = 0.6, 0.6, 0.6
        outline.color.a            = 0.9

        # B → A → C → D → B (closed rectangle)
        from geometry_msgs.msg import Point
        for n in ('B', 'A', 'C', 'D', 'B'):
            x, y = self.corners[n]
            p = Point()
            p.x, p.y, p.z = float(x), float(y), 0.02
            outline.points.append(p)

        arr.markers.append(outline)

        return arr


# ── Entry point ──────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = RoomCorners()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
