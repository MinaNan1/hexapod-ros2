"""
path_converter.py
=================
Converts a smoothed A* path (grid coordinates) into:
  1. Real-world metric coordinates (metres)
  2. Hexapod body_pose Twist commands  (tx, ty, tz, roll, pitch, yaw)

This is the bridge between "path planning" and "robot motion".

Coordinate conventions
-----------------------
Grid  : (row, col) — row increases downward, col increases rightward.
World : (x, y)     — x = col * resolution,  y = row * resolution  (metres).

Body pose (Twist)  — what hexapod_controller.py expects
------------------------
linear.x  = tx    (mm)  — forward translation per frame
linear.y  = ty    (mm)  — lateral translation per frame
linear.z  = tz    (mm)  — height (0 = nominal height)
angular.x = roll  (rad)
angular.y = pitch (rad)
angular.z = yaw   (rad) — body rotation per frame
"""

import math
import numpy as np
from typing import List, Tuple


class PathConverter:
    """
    Converts a grid path to world coordinates and hexapod motion commands.

    Parameters
    ----------
    map_resolution : float
        Metres per grid cell.  Typical: 0.05 m/cell for a 200×200 grid
        representing a 10 m × 10 m area.

    Usage
    -----
    converter = PathConverter(map_resolution=0.05)

    # Grid path → world coordinates
    world_path = converter.grid_to_world(grid_path)

    # World path → list of body_pose dicts
    commands = converter.path_to_commands(world_path)

    # Print all commands
    converter.print_commands(commands)
    """

    def __init__(self, map_resolution: float = 0.05):
        self.resolution = map_resolution  # m/cell

    # ------------------------------------------------------------------
    # Step 1 — Grid → World
    # ------------------------------------------------------------------

    def grid_to_world(
        self,
        path: List[Tuple[int, int]],
    ) -> List[Tuple[float, float]]:
        """
        Convert list of (row, col) grid positions to (x, y) in metres.

        Returns
        -------
        List of (x_metres, y_metres).
        """
        return [
            (col * self.resolution, row * self.resolution)
            for row, col in path
        ]

    # ------------------------------------------------------------------
    # Step 2 — World path → body_pose command sequence
    # ------------------------------------------------------------------

    def path_to_commands(
        self,
        world_path: List[Tuple[float, float]],
        step_speed_mm: float = 10.0,
        max_yaw_rad: float = 0.3,
    ) -> List[dict]:
        """
        Generate a list of body_pose command dicts for each path segment.

        Each dict matches the Twist fields expected by hexapod_controller:
            tx, ty, tz        (mm)
            roll, pitch, yaw  (rad)
            segment_length    (m) — informational
            heading_deg       (°) — informational

        Parameters
        ----------
        world_path    : list of (x, y) in metres.
        step_speed_mm : forward speed (mm per 50 ms control frame).
        max_yaw_rad   : maximum yaw rotation per frame (rad).
        """
        if len(world_path) < 2:
            return []

        commands = []
        current_yaw = 0.0  # robot starts facing +x direction

        for i in range(len(world_path) - 1):
            x0, y0 = world_path[i]
            x1, y1 = world_path[i + 1]

            dx = x1 - x0
            dy = y1 - y0
            segment_length = math.sqrt(dx ** 2 + dy ** 2)

            if segment_length < 1e-6:
                continue  # skip duplicate waypoints

            # Desired heading for this segment
            target_yaw = math.atan2(dy, dx)

            # Shortest angular difference
            yaw_error = self._angle_diff(target_yaw, current_yaw)
            yaw_cmd   = max(-max_yaw_rad, min(max_yaw_rad, yaw_error))
            current_yaw += yaw_cmd

            # Only move forward when mostly aligned (within ~17°)
            if abs(yaw_error) < 0.3:
                tx = step_speed_mm * math.cos(current_yaw)
                ty = step_speed_mm * math.sin(current_yaw)
            else:
                tx = ty = 0.0  # rotate in place first

            commands.append({
                'tx'             : round(tx,           4),
                'ty'             : round(ty,           4),
                'tz'             : 0.0,
                'roll'           : 0.0,
                'pitch'          : 0.0,
                'yaw'            : round(yaw_cmd,      4),
                'segment_length' : round(segment_length, 4),
                'heading_deg'    : round(math.degrees(target_yaw), 1),
            })

        return commands

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def print_commands(self, commands: List[dict], max_rows: int = 20) -> None:
        """Print a formatted preview of the command sequence."""
        print(f'Body-pose command sequence  ({len(commands)} segments)')
        print(f'{"#":>4}  {"tx":>7}  {"ty":>7}  {"yaw":>7}  '
              f'{"hdg°":>7}  {"seg_m":>7}')
        print('─' * 50)
        shown = min(len(commands), max_rows)
        for i, cmd in enumerate(commands[:shown]):
            print(f'{i+1:>4}  {cmd["tx"]:>7.2f}  {cmd["ty"]:>7.2f}  '
                  f'{cmd["yaw"]:>7.3f}  {cmd["heading_deg"]:>7.1f}  '
                  f'{cmd["segment_length"]:>7.4f}')
        if len(commands) > max_rows:
            print(f'  … {len(commands) - max_rows} more rows')

    @staticmethod
    def _angle_diff(target: float, current: float) -> float:
        """Return the smallest signed angle from current to target."""
        diff = target - current
        while diff >  math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi
        return diff
