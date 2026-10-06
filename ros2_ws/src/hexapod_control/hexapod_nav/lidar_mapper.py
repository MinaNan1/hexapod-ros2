"""
lidar_mapper.py
===============
ROS2 node — builds a live 2D occupancy grid from real LiDAR scans while the
hexapod walks around.

Subscribes:
    /scan  (sensor_msgs/LaserScan)  — real LDS-01 LiDAR data
    TF:    world ← laser            — looked up per scan to locate the LiDAR

Publishes:
    /map   (nav_msgs/OccupancyGrid) — live 2D occupancy grid in `world` frame

Pose-aware mapping (v3)
-----------------------
Earlier versions assumed the LiDAR was stationary at the world origin, which
worked on a table but smeared the map as soon as the hexapod moved.  This
version looks up `world ← laser` via TF at every scan and raycasts each beam
from wherever the laser actually is in the world.  The expected TF chain is:

    world  --static-->  odom  --dynamic(hexapod_controller)-->  base_link
                                                                   |
                                                                   +-- static --> laser

Other features retained:
    - Scan-level median filter rejects single-beam noise spikes.
    - Log-odds raycasting with Bresenham.
    - Optional temporal decay (default off — the map is persistent as the
      robot moves; re-enable with `decay_rate > 0` if you want a stationary
      LiDAR that forgets old observations).
    - No artificial dilation: the planner inflates separately.

Grid:
    200×200 cells, 0.05 m/cell = 10m × 10m area centered at world origin.

Usage:
    ros2 run hexapod_control lidar_mapper
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener, TransformException


class LidarMapper(Node):

    def __init__(self):
        super().__init__('lidar_mapper')

        # ── ROS2 Parameters ──────────────────────────────────────────
        self.declare_parameter('grid_width',        200)
        self.declare_parameter('grid_height',       200)
        self.declare_parameter('resolution',        0.05)   # m/cell
        self.declare_parameter('map_publish_rate',   2.0)   # Hz

        # Frames — TF is looked up per scan to locate the moving laser.
        self.declare_parameter('world_frame',       'world')
        self.declare_parameter('tf_timeout_sec',     0.1)

        # Log-odds tuning (v2 — more responsive)
        self.declare_parameter('log_odds_free',     -0.5)   # was -0.4: clear faster
        self.declare_parameter('log_odds_occupied',  0.85)  # was 0.9: accumulate slightly less
        self.declare_parameter('log_odds_max',       2.5)   # was 5.0: cap lower → easier to clear
        self.declare_parameter('log_odds_min',      -2.5)   # symmetric

        # Temporal decay — applied every publish cycle.  Default 0 so the
        # map is persistent as the robot moves.  If you want a stationary
        # LiDAR that forgets ghost walls, set e.g. 0.05.
        self.declare_parameter('decay_rate',         0.0)

        # Median-filter half-window for scan outlier rejection
        self.declare_parameter('scan_filter_window',  3)

        self.grid_w      = self.get_parameter('grid_width').value
        self.grid_h      = self.get_parameter('grid_height').value
        self.res         = self.get_parameter('resolution').value
        pub_rate         = self.get_parameter('map_publish_rate').value
        self.world_frame = self.get_parameter('world_frame').value
        self.tf_timeout  = self.get_parameter('tf_timeout_sec').value
        self.l_free      = self.get_parameter('log_odds_free').value
        self.l_occ       = self.get_parameter('log_odds_occupied').value
        self.l_max       = self.get_parameter('log_odds_max').value
        self.l_min       = self.get_parameter('log_odds_min').value
        self.decay_rate  = self.get_parameter('decay_rate').value
        self.filter_win  = self.get_parameter('scan_filter_window').value

        # ── Grid origin — centre at world (0, 0) ─────────────────────
        self.origin_x = -self.grid_w * self.res / 2.0   # -5.0 m
        self.origin_y = -self.grid_h * self.res / 2.0   # -5.0 m

        # Log-odds grid (0.0 = unknown prior)
        self.log_odds = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)

        # ── TF listener — locates the laser in world frame each scan ─
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── ROS2 Interfaces ──────────────────────────────────────────
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.create_timer(1.0 / pub_rate, self._publish_map)

        self._scan_count = 0
        self._tf_miss    = 0
        self.get_logger().info(
            f'LidarMapper v3 ready — {self.grid_w}×{self.grid_h} grid '
            f'({self.grid_w * self.res:.0f}m × {self.grid_h * self.res:.0f}m), '
            f'res={self.res} m/cell, world={self.world_frame}, '
            f'decay={self.decay_rate}')

    # ──────────────────────────────────────────────────────────────────
    # Scan callback
    # ──────────────────────────────────────────────────────────────────

    def _on_scan(self, msg: LaserScan):
        """Process one LaserScan and update the occupancy grid.

        Each beam's endpoint is transformed from the laser frame into the
        world frame using the current TF, then raycast from the laser's
        grid cell — so the map accumulates correctly as the hexapod walks.
        """
        # ── Step 1: Look up laser pose in world frame ───────────────
        pose = self._laser_pose_in_world(msg.header)
        if pose is None:
            return  # TF not ready yet — drop scan (warning logged inside)

        laser_x, laser_y, laser_yaw = pose

        # Convert laser position to grid coords (start of every ray)
        laser_col = int((laser_x - self.origin_x) / self.res)
        laser_row = int((laser_y - self.origin_y) / self.res)

        if not (0 <= laser_col < self.grid_w and 0 <= laser_row < self.grid_h):
            # Robot has walked outside the mapped area — skip silently
            return

        self._scan_count += 1

        ranges = np.array(msg.ranges, dtype=np.float32)

        # ── Step 2: Median filter to reject outlier spikes ───────────
        # Replace each reading with the median of a small window around
        # it.  This eliminates single-sample noise spikes that would
        # otherwise create phantom obstacle dots on the map.
        filtered = self._median_filter_ranges(ranges, msg.range_min, msg.range_max)

        # ── Step 3: Ray-cast each valid beam in world frame ──────────
        for i, r in enumerate(filtered):
            # Beam angle in laser frame, rotated into world frame by yaw
            angle = msg.angle_min + i * msg.angle_increment + laser_yaw

            # Skip invalid / NaN / inf
            if math.isnan(r) or math.isinf(r) or r < msg.range_min:
                continue

            is_max_range = (r >= msg.range_max)
            effective_r  = min(r, msg.range_max)

            # Endpoint in world frame (laser at laser_x, laser_y)
            end_x = laser_x + effective_r * math.cos(angle)
            end_y = laser_y + effective_r * math.sin(angle)

            # Convert to grid coordinates
            end_col = int((end_x - self.origin_x) / self.res)
            end_row = int((end_y - self.origin_y) / self.res)

            # Ray-trace and update log-odds
            self._ray_trace(
                laser_col, laser_row,
                end_col, end_row,
                mark_endpoint=(not is_max_range),
            )

        if self._scan_count % 10 == 0:
            occ  = int(np.sum(self.log_odds > 0.5))
            free = int(np.sum(self.log_odds < -0.5))
            self.get_logger().info(
                f'Scan #{self._scan_count} | pose=({laser_x:+.2f},{laser_y:+.2f},'
                f'{math.degrees(laser_yaw):+.0f}°) | '
                f'occupied={occ} free={free} cells')

    # ──────────────────────────────────────────────────────────────────
    # TF lookup: where is the laser right now in the world frame?
    # ──────────────────────────────────────────────────────────────────

    def _laser_pose_in_world(self, scan_header):
        """Look up (x, y, yaw) of the laser in the world frame at scan time.

        Returns None if the transform is not yet available — the scan is
        then dropped rather than raycast from a stale or wrong pose.
        """
        laser_frame = scan_header.frame_id or 'laser'
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                laser_frame,
                scan_header.stamp,
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as ex:
            self._tf_miss += 1
            if self._tf_miss == 1 or self._tf_miss % 20 == 0:
                self.get_logger().warn(
                    f'TF {self.world_frame} ← {laser_frame} not available '
                    f'({self._tf_miss} scans dropped): {ex}')
            return None

        self._tf_miss = 0

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y

        # Yaw from quaternion (planar robot — roll/pitch ignored)
        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        return tx, ty, yaw

    # ──────────────────────────────────────────────────────────────────
    # Scan outlier filter
    # ──────────────────────────────────────────────────────────────────

    def _median_filter_ranges(self, ranges, range_min, range_max):
        """Apply a sliding-window median filter to reject spike noise.

        A reading that is wildly different from its angular neighbours
        (e.g. a single 0.3 m spike surrounded by 3.0 m readings) is
        almost certainly sensor noise.  Replacing it with the local
        median removes these phantom dots from the map.
        """
        n = len(ranges)
        if n == 0 or self.filter_win <= 0:
            return ranges

        filtered = ranges.copy()
        half_w = self.filter_win

        for i in range(n):
            lo = max(0, i - half_w)
            hi = min(n, i + half_w + 1)
            window = ranges[lo:hi]
            # Only consider valid readings in the window
            valid = window[(window >= range_min) & (window <= range_max)
                           & ~np.isnan(window) & ~np.isinf(window)]
            if len(valid) >= 2:
                med = np.median(valid)
                # If this reading deviates by more than 40% from median,
                # replace it with the median — it's likely noise.
                if not math.isnan(ranges[i]) and ranges[i] >= range_min:
                    if abs(ranges[i] - med) > 0.4 * med:
                        filtered[i] = med

        return filtered

    # ──────────────────────────────────────────────────────────────────
    # Ray tracing (Bresenham + log-odds update)
    # ──────────────────────────────────────────────────────────────────

    def _ray_trace(self, x0, y0, x1, y1, mark_endpoint=True):
        """Bresenham ray from (x0,y0) to (x1,y1).
        Intermediate cells → free, endpoint → occupied."""
        cells = self._bresenham(x0, y0, x1, y1)

        for i, (cx, cy) in enumerate(cells):
            if not (0 <= cx < self.grid_w and 0 <= cy < self.grid_h):
                continue

            if i == len(cells) - 1 and mark_endpoint:
                self.log_odds[cy, cx] = np.clip(
                    self.log_odds[cy, cx] + self.l_occ, self.l_min, self.l_max)
            else:
                self.log_odds[cy, cx] = np.clip(
                    self.log_odds[cy, cx] + self.l_free, self.l_min, self.l_max)

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        """Bresenham's line algorithm → list of (x, y) cell coords."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return cells

    # ──────────────────────────────────────────────────────────────────
    # Temporal decay
    # ──────────────────────────────────────────────────────────────────

    def _apply_decay(self):
        """Shrink all log-odds values toward zero (unknown).

        This is the key fix for the "ghost wall" problem:
        - Cells that are actively observed (hit by current scans) get
          continuously reinforced and remain black/white.
        - Cells that are NO LONGER observed (because the LiDAR was
          rotated) stop being reinforced.  The decay gradually pulls
          them back to 0 (unknown → grey), erasing the ghosts.

        The decay is subtractive (not multiplicative) so that it works
        the same regardless of the current log-odds magnitude.
        """
        # Decay occupied cells (positive log-odds) toward zero
        pos_mask = self.log_odds > 0
        self.log_odds[pos_mask] = np.maximum(
            0.0, self.log_odds[pos_mask] - self.decay_rate)

        # Decay free cells (negative log-odds) toward zero
        neg_mask = self.log_odds < 0
        self.log_odds[neg_mask] = np.minimum(
            0.0, self.log_odds[neg_mask] + self.decay_rate)

    # ──────────────────────────────────────────────────────────────────
    # Map publisher
    # ──────────────────────────────────────────────────────────────────

    def _publish_map(self):
        """Apply decay, convert log-odds → OccupancyGrid, publish."""
        # ── Decay old readings ───────────────────────────────────────
        self._apply_decay()

        # ── Build OccupancyGrid message ──────────────────────────────
        msg = OccupancyGrid()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame

        msg.info.resolution = float(self.res)
        msg.info.width      = self.grid_w
        msg.info.height     = self.grid_h
        msg.info.origin.position.x    = self.origin_x
        msg.info.origin.position.y    = self.origin_y
        msg.info.origin.position.z    = 0.0
        msg.info.origin.orientation.w = 1.0

        # Convert log-odds → OccupancyGrid values
        #   -1 = unknown (grey in RViz)
        #    0 = definitely free (white in RViz)
        #  100 = definitely occupied (black in RViz)
        #
        # No dilation — show true obstacle positions for accuracy.
        # The path planner inflates obstacles separately for safety.
        flat = self.log_odds.flatten()
        data = np.full(flat.shape, -1, dtype=np.int8)   # default: unknown

        # Free space: log-odds clearly negative
        data[flat < -0.3] = 0

        # Occupied: log-odds clearly positive — scale to 50-100 range
        occ_mask = flat > 0.5
        data[occ_mask] = np.clip(
            (50 + (flat[occ_mask] / self.l_max) * 50).astype(np.int8),
            50, 100)

        msg.data = data.tolist()
        self.map_pub.publish(msg)


# ── Entry point ──────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LidarMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
