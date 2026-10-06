"""
frontier_explorer.py
====================
Identifies frontier cells (edges between known and unknown space) and
selects the next exploration goal.

Frontier = cells that are free (0) and adjacent to unknown (-1) cells.
This is where the robot should explore next to expand its knowledge map.

Usage
-----
explorer = FrontierExplorer(known_map, current_pos, inflated_grid)
frontier_cells = explorer.find_frontier()
next_goal = explorer.pick_nearest_frontier(current_pos)
"""

import numpy as np
from collections import deque
from typing import List, Tuple, Optional


class FrontierExplorer:
    """
    Identifies frontier cells in a partially-known map.

    Map convention:
        -1  = unknown (not yet revealed)
         0  = known free space
         1  = known obstacle
    
    A frontier cell is a free cell (0) adjacent to unknown cells (-1).
    """

    def __init__(self, known_map: np.ndarray, inflated_grid: np.ndarray, 
                 robot_radius_cells: int = 2):
        """
        Initialize the explorer.

        Args:
            known_map: Occupancy grid with -1 (unknown), 0 (free), 1 (obstacle)
            inflated_grid: Inflated obstacle map for collision checking
            robot_radius_cells: Robot body radius in cells (for validation)
        """
        self.known_map = known_map
        self.inflated_grid = inflated_grid
        self.robot_radius_cells = robot_radius_cells
        self.h, self.w = known_map.shape

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def find_frontier(self) -> List[Tuple[int, int]]:
        """
        Find all frontier cells (free cells adjacent to unknown).

        Returns
        -------
        List of (row, col) tuples representing frontier cells.
        """
        frontier = []

        # 8-directional neighbors for frontier detection
        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]

        for r in range(self.h):
            for c in range(self.w):
                # Check if this cell is free
                if self.known_map[r, c] == 0:
                    # Check if it's adjacent to unknown
                    has_unknown_neighbor = False
                    for dr, dc in directions:
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < self.h and 0 <= nc < self.w:
                            if self.known_map[nr, nc] == -1:
                                has_unknown_neighbor = True
                                break
                    
                    if has_unknown_neighbor:
                        frontier.append((r, c))

        return frontier

    def pick_nearest_frontier(self, current_pos: Tuple[int, int], 
                             frontier_cells: Optional[List[Tuple[int, int]]] = None) \
            -> Optional[Tuple[int, int]]:
        """
        Select the nearest frontier cell to current position.

        Args:
            current_pos: (row, col) of robot
            frontier_cells: Pre-computed frontier. If None, calls find_frontier()

        Returns
        -------
        (row, col) of nearest frontier cell, or None if no frontier exists.
        """
        if frontier_cells is None:
            frontier_cells = self.find_frontier()

        if not frontier_cells:
            return None

        # Find nearest using Euclidean distance
        min_dist = float('inf')
        nearest = None

        for frontier_cell in frontier_cells:
            dist = np.sqrt((frontier_cell[0] - current_pos[0])**2 + 
                          (frontier_cell[1] - current_pos[1])**2)
            if dist < min_dist:
                min_dist = dist
                nearest = frontier_cell

        return nearest

    def pick_random_frontier(self, frontier_cells: Optional[List[Tuple[int, int]]] = None) \
            -> Optional[Tuple[int, int]]:
        """
        Select a random frontier cell (tie-breaker for clusters).

        Args:
            frontier_cells: Pre-computed frontier. If None, calls find_frontier()

        Returns
        -------
        Random frontier cell, or None if no frontier exists.
        """
        if frontier_cells is None:
            frontier_cells = self.find_frontier()

        if not frontier_cells:
            return None

        return frontier_cells[np.random.randint(len(frontier_cells))]

    def get_frontier_stats(self) -> dict:
        """
        Get statistics about frontier and unknown space.

        Returns
        -------
        Dictionary with:
            - total_cells: Total grid size
            - unknown_cells: Count of unknown cells
            - known_free_cells: Count of known free cells
            - known_obstacle_cells: Count of obstacles
            - frontier_cells: Count of frontier cells
            - frontier_percentage: % of map that is frontier
        """
        frontier = self.find_frontier()

        unknown = np.sum(self.known_map == -1)
        known_free = np.sum(self.known_map == 0)
        known_obstacle = np.sum(self.known_map == 1)
        total = self.h * self.w

        return {
            'total_cells': total,
            'unknown_cells': int(unknown),
            'known_free_cells': int(known_free),
            'known_obstacle_cells': int(known_obstacle),
            'frontier_cells': len(frontier),
            'frontier_percentage': round(100 * len(frontier) / total, 1),
            'unknown_percentage': round(100 * unknown / total, 1),
        }

    # ------------------------------------------------------------------
    # Visualization helper
    # ------------------------------------------------------------------

    def visualize_frontier(self, current_pos: Tuple[int, int],
                          frontier_cells: Optional[List[Tuple[int, int]]] = None,
                          goal_pos: Optional[Tuple[int, int]] = None):
        """
        Print text visualization of map with frontier marked.

        Args:
            current_pos: Robot position (row, col)
            frontier_cells: Frontier cells to highlight
            goal_pos: Goal position (row, col)
        """
        if frontier_cells is None:
            frontier_cells = self.find_frontier()

        frontier_set = set(frontier_cells)

        # Compress for visualization (show only center portion)
        h_min, h_max = max(0, current_pos[0] - 20), min(self.h, current_pos[0] + 20)
        c_min, c_max = max(0, current_pos[1] - 20), min(self.w, current_pos[1] + 20)

        print('\n── Frontier Map ───────────────────────────')
        for r in range(h_min, h_max):
            for c in range(c_min, c_max):
                if (r, c) == current_pos:
                    print('R', end=' ')  # Robot
                elif (r, c) == goal_pos:
                    print('G', end=' ')  # Goal
                elif (r, c) in frontier_set:
                    print('F', end=' ')  # Frontier
                elif self.known_map[r, c] == -1:
                    print('?', end=' ')  # Unknown
                elif self.known_map[r, c] == 0:
                    print('.', end=' ')  # Free
                else:
                    print('#', end=' ')  # Obstacle
            print()
        print('───────────────────────────────────────────')

    # ------------------------------------------------------------------
    # Path validation (check if path crosses unknown)
    # ------------------------------------------------------------------

    def is_path_valid(self, path: List[Tuple[int, int]]) -> bool:
        """
        Check if a planned path still avoids obstacles and unknown cells.

        Args:
            path: List of (row, col) waypoints

        Returns
        -------
        True if path is valid (all cells are known free or unrevealed-but-clear)
        False if path hits a revealed obstacle
        """
        for r, c in path:
            if not (0 <= r < self.h and 0 <= c < self.w):
                return False
            # Obstacle blocks path
            if self.known_map[r, c] == 1:
                return False

        return True

    def find_path_blockage(self, path: List[Tuple[int, int]]) -> Optional[Tuple[int, int]]:
        """
        Find the first obstacle cell in a path (if any).

        Args:
            path: List of (row, col) waypoints

        Returns
        -------
        (row, col) of first obstacle found, or None if path is clear.
        """
        for r, c in path:
            if 0 <= r < self.h and 0 <= c < self.w:
                if self.known_map[r, c] == 1:
                    return (r, c)

        return None
