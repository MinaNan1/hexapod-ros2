"""
path_smoother.py
================
Smooths the raw A* waypoint list before feeding it to the hexapod.

Why smooth?
-----------
A* on a grid produces a staircase path — lots of tiny diagonal/cardinal
steps. Sending every single grid cell as a waypoint to the hexapod would
cause excessive direction changes and jerky motion.

Two smoothing strategies are provided:

1. rdp_simplify   — Ramer–Douglas–Peucker line simplification.
                    Removes redundant collinear points, keeping only
                    waypoints where the path actually turns.
                    Fast and simple. Recommended first choice.

2. interpolate    — Linear interpolation between a decimated set of
                    keypoints. Useful when you want a fixed number of
                    evenly-spaced waypoints.
"""

import numpy as np
from typing import List, Tuple


class PathSmoother:
    """
    Simplifies and smooths an A* path.

    Usage
    -----
    smoother = PathSmoother()

    # Remove redundant collinear points (recommended)
    simplified = smoother.rdp_simplify(raw_path, epsilon=2.0)

    # Linear interpolation (fixed spacing)
    interp = smoother.interpolate(simplified, spacing=3)
    """

    # ------------------------------------------------------------------
    # RDP simplification
    # ------------------------------------------------------------------

    def rdp_simplify(
        self,
        path: List[Tuple[int, int]],
        epsilon: float = 2.0,
    ) -> List[Tuple[int, int]]:
        """
        Ramer–Douglas–Peucker simplification.

        Recursively removes points that deviate less than `epsilon` cells
        from the straight line connecting their neighbours.

        Parameters
        ----------
        path    : list of (row, col)
        epsilon : tolerance in grid cells.
                  Smaller → keep more points (closer to original).
                  Larger  → more aggressive simplification.
                  Typical range: 1.0 – 5.0

        Returns
        -------
        Simplified path as list of (row, col).
        """
        if len(path) <= 2:
            return list(path)
        return self._rdp_recursive(path, epsilon)

    # ------------------------------------------------------------------
    # Linear interpolation
    # ------------------------------------------------------------------

    def interpolate(
        self,
        path: List[Tuple[int, int]],
        spacing: float = 5.0,
    ) -> List[Tuple[int, int]]:
        """
        Re-sample path with evenly-spaced waypoints.

        Parameters
        ----------
        path    : list of (row, col)
        spacing : desired distance between consecutive waypoints (cells).
                  Smaller → denser waypoints.

        Returns
        -------
        Re-sampled path as list of (row, col).
        """
        if len(path) < 2:
            return list(path)

        arr = np.array(path, dtype=float)

        # Cumulative arc length along the path
        dists  = np.linalg.norm(np.diff(arr, axis=0), axis=1)
        cumlen = np.concatenate([[0], np.cumsum(dists)])
        total  = cumlen[-1]

        if total == 0:
            return list(path)

        # Target sample positions
        targets = np.arange(0, total, spacing)
        if targets[-1] < total:
            targets = np.append(targets, total)

        # Interpolate row and col separately
        rows = np.interp(targets, cumlen, arr[:, 0])
        cols = np.interp(targets, cumlen, arr[:, 1])

        return [(int(round(r)), int(round(c))) for r, c in zip(rows, cols)]

    # ------------------------------------------------------------------
    # Stats helper
    # ------------------------------------------------------------------

    @staticmethod
    def stats(original: list, smoothed: list) -> None:
        """Print before/after waypoint counts."""
        ratio = len(smoothed) / max(len(original), 1)
        print(f'  Waypoints: {len(original)} → {len(smoothed)} '
              f'({ratio:.1%} of original)')

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _rdp_recursive(
        self,
        points: List[Tuple[int, int]],
        epsilon: float,
    ) -> List[Tuple[int, int]]:
        """Recursive RDP implementation."""
        if len(points) <= 2:
            return list(points)

        start = np.array(points[0], dtype=float)
        end   = np.array(points[-1], dtype=float)

        # Perpendicular distances from all intermediate points to start–end line
        max_dist  = 0.0
        max_index = 0

        for i in range(1, len(points) - 1):
            pt   = np.array(points[i], dtype=float)
            dist = self._point_to_line_dist(pt, start, end)
            if dist > max_dist:
                max_dist  = dist
                max_index = i

        if max_dist > epsilon:
            # Split at the farthest point and recurse both halves
            left  = self._rdp_recursive(points[:max_index + 1], epsilon)
            right = self._rdp_recursive(points[max_index:],     epsilon)
            return left[:-1] + right  # avoid duplicate junction point
        else:
            # All intermediate points are close enough → discard them
            return [points[0], points[-1]]

    @staticmethod
    def _point_to_line_dist(
        pt: np.ndarray,
        line_start: np.ndarray,
        line_end: np.ndarray,
    ) -> float:
        """Perpendicular distance from pt to the line through line_start→line_end."""
        seg = line_end - line_start
        seg_len_sq = float(np.dot(seg, seg))

        if seg_len_sq == 0:
            return float(np.linalg.norm(pt - line_start))

        # Project pt onto the line
        t = float(np.dot(pt - line_start, seg) / seg_len_sq)
        t = max(0.0, min(1.0, t))
        projection = line_start + t * seg
        return float(np.linalg.norm(pt - projection))
