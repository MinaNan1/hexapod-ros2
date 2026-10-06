"""
reactive_planner.py
===================
Handles reactive replanning when obstacles block the robot's current path.

Strategy (Hybrid replanning):
  1. When path is blocked, check if we can detour around the obstacle
  2. If detour is possible (< N cells away), use it locally
  3. If detour is impossible or costly, trigger full A* replan to next goal

This reduces unnecessary global replans while still reacting to new obstacles.

Usage
-----
reactor = ReactivePlanner(inflated_grid)
can_detour = reactor.find_detour(path, blockage_pos)
if can_detour:
    new_path = reactor.apply_detour(path, blockage_pos, detour_path)
else:
    # Trigger full A* replan
"""

import numpy as np
from heapq import heappush, heappop
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


@dataclass(order=True)
class _Node:
    """Simple node for local search."""
    f: float
    count: int
    g: float = field(compare=False)
    row: int = field(compare=False)
    col: int = field(compare=False)
    parent: object = field(compare=False, default=None)


DIRECTIONS = [
    (-1,  0, 1.0),   # North
    ( 1,  0, 1.0),   # South
    ( 0, -1, 1.0),   # West
    ( 0,  1, 1.0),   # East
    (-1, -1, 1.414), # NW diagonal
    (-1,  1, 1.414), # NE diagonal
    ( 1, -1, 1.414), # SW diagonal
    ( 1,  1, 1.414), # SE diagonal
]


class ReactivePlanner:
    """
    Handles local replanning when obstacles block the current path.
    
    Uses local A* to find detours around newly-discovered obstacles.
    """

    def __init__(self, inflated_grid: np.ndarray, max_detour_cost: float = 20.0):
        """
        Initialize reactive planner.

        Args:
            inflated_grid: Occupancy grid (0=free, 1=obstacle)
            max_detour_cost: Maximum acceptable detour cost before triggering
                           full global replan (in cells)
        """
        self.inflated_grid = inflated_grid
        self.max_detour_cost = max_detour_cost
        self.h, self.w = inflated_grid.shape
        self._counter = 0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def find_detour(self, current_path: List[Tuple[int, int]],
                   blockage_pos: Tuple[int, int],
                   search_radius: int = 10) -> Optional[List[Tuple[int, int]]]:
        """
        Attempt to find a detour around a blocked point.

        Strategy:
          1. Find the waypoint BEFORE the blockage
          2. Find the waypoint AFTER the blockage
          3. Use local A* to connect them, avoiding the blockage area

        Args:
            current_path: Current planned path (list of waypoints)
            blockage_pos: (row, col) of newly-discovered obstacle
            search_radius: How many cells around blockage to search for detour

        Returns
        -------
        Detour path (list of waypoints) if found, None if no good detour exists.
        """
        # Find which waypoint in path is closest to blockage
        blockage_idx = None
        min_dist = float('inf')

        for i, wp in enumerate(current_path):
            dist = abs(wp[0] - blockage_pos[0]) + abs(wp[1] - blockage_pos[1])
            if dist < min_dist:
                min_dist = dist
                blockage_idx = i

        if blockage_idx is None or blockage_idx == 0:
            return None

        # Connect waypoint before blockage to waypoint after blockage
        before = current_path[blockage_idx - 1]
        after = current_path[min(blockage_idx + 1, len(current_path) - 1)]

        # Local A* from before → after, with blockage area marked as penalty
        detour = self._local_astar(before, after, blockage_pos, search_radius)

        return detour

    def apply_detour(self, current_path: List[Tuple[int, int]],
                    blockage_pos: Tuple[int, int],
                    detour_path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """
        Splice detour into current path, removing the blocked segment.

        Args:
            current_path: Original planned path
            blockage_pos: Position of obstacle
            detour_path: Detour waypoints

        Returns
        -------
        New path with detour spliced in.
        """
        blockage_idx = None
        min_dist = float('inf')

        for i, wp in enumerate(current_path):
            dist = abs(wp[0] - blockage_pos[0]) + abs(wp[1] - blockage_pos[1])
            if dist < min_dist:
                min_dist = dist
                blockage_idx = i

        if blockage_idx is None:
            return current_path

        # Keep everything before blockage, add detour, keep everything after
        new_path = (current_path[:blockage_idx] + 
                   detour_path + 
                   current_path[blockage_idx + 1:])

        return new_path

    def estimate_detour_cost(self, detour_path: List[Tuple[int, int]],
                            direct_distance: float) -> float:
        """
        Estimate how costly the detour is compared to direct path.

        Args:
            detour_path: Detour waypoints
            direct_distance: Euclidean distance of direct path

        Returns
        -------
        Cost ratio (> 1.0 means detour is longer)
        """
        if len(detour_path) < 2:
            return 1.0

        detour_length = 0.0
        for i in range(len(detour_path) - 1):
            r1, c1 = detour_path[i]
            r2, c2 = detour_path[i + 1]
            detour_length += np.sqrt((r2 - r1)**2 + (c2 - c1)**2)

        if direct_distance < 0.1:
            return 1.0

        return detour_length / direct_distance

    # ------------------------------------------------------------------
    # Internal: Local A*
    # ------------------------------------------------------------------

    def _local_astar(self, start: Tuple[int, int], goal: Tuple[int, int],
                     blockage_pos: Tuple[int, int], search_radius: int) \
            -> Optional[List[Tuple[int, int]]]:
        """
        Local A* search from start to goal, avoiding blockage area.

        Args:
            start: (row, col) start position
            goal: (row, col) goal position
            blockage_pos: (row, col) of obstacle to avoid
            search_radius: Search region around blockage

        Returns
        -------
        Path if found, None otherwise.
        """
        # Check bounds
        if not (0 <= start[0] < self.h and 0 <= start[1] < self.w):
            return None
        if not (0 <= goal[0] < self.h and 0 <= goal[1] < self.w):
            return None

        open_set = []
        closed_set = set()
        g_scores = {}
        parents = {}

        start_h = self._heuristic(start, goal)
        self._counter += 1
        heappush(open_set, _Node(start_h, self._counter, 0.0, start[0], start[1]))
        g_scores[start] = 0.0

        while open_set:
            node = heappop(open_set)
            pos = (node.row, node.col)

            if pos in closed_set:
                continue

            if pos == goal:
                # Reconstruct path
                path = []
                current = goal
                while current in parents:
                    path.append(current)
                    current = parents[current]
                path.append(start)
                return list(reversed(path))

            closed_set.add(pos)

            # Expand neighbors
            for dr, dc, move_cost in DIRECTIONS:
                nr, nc = node.row + dr, node.col + dc
                neighbor = (nr, nc)

                if not (0 <= nr < self.h and 0 <= nc < self.w):
                    continue
                if neighbor in closed_set:
                    continue

                # Don't cross obstacles
                if self.inflated_grid[nr, nc] == 1:
                    continue

                # Penalize cells near blockage (soft constraint)
                penalty = 0.0
                dist_to_blockage = max(
                    abs(nr - blockage_pos[0]),
                    abs(nc - blockage_pos[1])
                )
                if dist_to_blockage <= search_radius:
                    penalty = (search_radius - dist_to_blockage) * 0.5

                tentative_g = g_scores[pos] + move_cost + penalty

                if neighbor not in g_scores or tentative_g < g_scores[neighbor]:
                    parents[neighbor] = pos
                    g_scores[neighbor] = tentative_g
                    h = self._heuristic(neighbor, goal)
                    f = tentative_g + h
                    self._counter += 1
                    heappush(open_set, _Node(f, self._counter, tentative_g, nr, nc))

        return None  # No path found

    def _heuristic(self, pos: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """Euclidean distance heuristic."""
        return np.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)


class PathMonitor:
    """
    Monitors a path and detects when it becomes invalid.
    
    Used alongside ReactivePlanner to detect blockages early.
    """

    def __init__(self, inflated_grid: np.ndarray):
        self.inflated_grid = inflated_grid
        self.h, self.w = inflated_grid.shape
        self.monitored_path = []
        self.waypoint_idx = 0

    def set_path(self, path: List[Tuple[int, int]]):
        """Set a new path to monitor."""
        self.monitored_path = path
        self.waypoint_idx = 0

    def advance(self, current_pos: Tuple[int, int]) -> bool:
        """
        Check if robot has progressed along path without hitting obstacles.

        Args:
            current_pos: Current robot position (row, col)

        Returns
        -------
        True if path is still valid, False if blockage detected.
        """
        if not self.monitored_path:
            return True

        # Check if current position is on a known obstacle
        if (0 <= current_pos[0] < self.h and 
            0 <= current_pos[1] < self.w and
            self.inflated_grid[current_pos[0], current_pos[1]] == 1):
            return False

        # Update waypoint progress
        min_dist = float('inf')
        closest_idx = self.waypoint_idx

        for i in range(self.waypoint_idx, len(self.monitored_path)):
            wp = self.monitored_path[i]
            dist = np.sqrt((wp[0] - current_pos[0])**2 + (wp[1] - current_pos[1])**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        self.waypoint_idx = closest_idx

        return True

    def get_next_waypoint(self) -> Optional[Tuple[int, int]]:
        """Get the next waypoint to head toward."""
        if self.waypoint_idx < len(self.monitored_path):
            return self.monitored_path[self.waypoint_idx]
        return None
