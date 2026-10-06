"""
scan_odom_node.py
=================
Publishes odom -> base_link TF by matching consecutive laser scans.
Gives SLAM Toolbox a real odometry prior instead of a static transform.

Algorithm: SVD-based rigid 2D registration (Kabsch) on angle-indexed
point correspondences between consecutive scans.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros


class ScanOdomNode(Node):
    def __init__(self):
        super().__init__('scan_odom_node')

        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Accumulated pose (global frame)
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0

        # Previous scan data
        self._prev_points = None
        self._prev_valid = None

        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        # Publish TF at 50Hz for smooth tracking
        self.create_timer(0.02, self._publish_tf)

        self.get_logger().info('ScanOdomNode ready — laser odometry active')

    def _scan_to_points(self, msg):
        """Convert LaserScan to Nx2 array of (x, y) points."""
        n = len(msg.ranges)
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        ranges = np.array(msg.ranges, dtype=np.float64)

        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        return np.column_stack([x, y]), valid

    def _estimate_transform(self, pts_prev, pts_curr, v_prev, v_curr):
        """Estimate 2D rigid transform between two scans using SVD."""
        valid = v_prev & v_curr
        if np.sum(valid) < 30:
            return 0.0, 0.0, 0.0

        p = pts_prev[valid]
        q = pts_curr[valid]

        p_mean = p.mean(axis=0)
        q_mean = q.mean(axis=0)

        H = (p - p_mean).T @ (q - q_mean)
        U, _, Vt = np.linalg.svd(H)

        d = np.linalg.det(Vt.T @ U.T)
        R = Vt.T @ np.array([[1, 0], [0, d]]) @ U.T
        t = q_mean - R @ p_mean

        # Robot motion = inverse of point-cloud shift
        dtheta = -math.atan2(R[1, 0], R[0, 0])
        displacement = -R.T @ t
        dx, dy = displacement[0], displacement[1]

        # Reject outlier transforms (too large for one frame)
        if abs(dx) > 0.15 or abs(dy) > 0.15 or abs(dtheta) > 0.3:
            return 0.0, 0.0, 0.0

        return dx, dy, dtheta

    def _scan_cb(self, msg):
        points, valid = self._scan_to_points(msg)

        if self._prev_points is not None:
            dx, dy, dtheta = self._estimate_transform(
                self._prev_points, points, self._prev_valid, valid)

            # Accumulate in global frame
            cos_t = math.cos(self._theta)
            sin_t = math.sin(self._theta)
            self._x += cos_t * dx - sin_t * dy
            self._y += sin_t * dx + cos_t * dy
            self._theta += dtheta

        self._prev_points = points
        self._prev_valid = valid

    def _publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self._theta / 2.0)
        t.transform.rotation.w = math.cos(self._theta / 2.0)
        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ScanOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
