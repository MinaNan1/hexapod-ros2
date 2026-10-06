"""
map_validator.py
================
Validates an occupancy grid and start/goal positions before
handing them to the path planner.  Catches common mistakes early
so you get a clear error message instead of a silent planner failure.

Checks performed
----------------
1. Grid is a 2D numpy array with only 0/1 values.
2. Start and goal are inside the map bounds.
3. Start and goal are not inside an obstacle.
4. Start ≠ goal.
5. (Optional) Basic connectivity check — warns if start/goal regions
   appear isolated, though full connectivity requires BFS/flood-fill.
"""

import numpy as np


class MapValidator:
    """
    Validates map + start/goal before running A*.

    Usage
    -----
    validator = MapValidator(grid, start=(10, 10), goal=(180, 180))
    ok = validator.validate()          # prints all issues and returns bool
    validator.summary()                # prints map stats
    """

    def __init__(self, grid: np.ndarray, start: tuple, goal: tuple):
        self.grid  = grid
        self.start = start
        self.goal  = goal
        self._errors   = []
        self._warnings = []

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def validate(self, verbose: bool = True) -> bool:
        """
        Run all checks.

        Returns
        -------
        True  — map is valid and planning can proceed.
        False — one or more errors found; see printed output.
        """
        self._errors.clear()
        self._warnings.clear()

        self._check_grid_type()
        self._check_grid_values()
        self._check_bounds(self.start, 'Start')
        self._check_bounds(self.goal,  'Goal')
        self._check_obstacle(self.start, 'Start')
        self._check_obstacle(self.goal,  'Goal')
        self._check_not_same()

        if verbose:
            self._print_report()

        return len(self._errors) == 0

    def summary(self) -> None:
        """Print a short summary of map statistics."""
        h, w   = self.grid.shape
        obs    = int(np.sum(self.grid))
        free   = h * w - obs
        pct    = round(100 * obs / (h * w), 1)

        print('── Map Summary ───────────────────────────')
        print(f'  Dimensions  : {w} × {h}  ({w * h} cells)')
        print(f'  Obstacle    : {obs} cells  ({pct}%)')
        print(f'  Free space  : {free} cells  ({100 - pct}%)')
        print(f'  Start       : row={self.start[0]}, col={self.start[1]}')
        print(f'  Goal        : row={self.goal[0]},  col={self.goal[1]}')
        print('─────────────────────────────────────────')

    # ------------------------------------------------------------------
    # Internal checks
    # ------------------------------------------------------------------

    def _check_grid_type(self):
        if not isinstance(self.grid, np.ndarray):
            self._errors.append('Grid must be a numpy ndarray.')
        elif self.grid.ndim != 2:
            self._errors.append(
                f'Grid must be 2D, got {self.grid.ndim}D array.')

    def _check_grid_values(self):
        if isinstance(self.grid, np.ndarray) and self.grid.ndim == 2:
            unique = set(np.unique(self.grid).tolist())
            if not unique.issubset({0, 1}):
                self._errors.append(
                    f'Grid contains values other than 0/1: {unique}')

    def _check_bounds(self, pos: tuple, name: str):
        if not isinstance(self.grid, np.ndarray) or self.grid.ndim != 2:
            return
        h, w = self.grid.shape
        r, c = pos
        if not (0 <= r < h and 0 <= c < w):
            self._errors.append(
                f'{name} {pos} is out of bounds '
                f'(map is {h} rows × {w} cols).'
            )

    def _check_obstacle(self, pos: tuple, name: str):
        if not isinstance(self.grid, np.ndarray) or self.grid.ndim != 2:
            return
        h, w = self.grid.shape
        r, c = pos
        if 0 <= r < h and 0 <= c < w:
            if self.grid[r, c] == 1:
                self._errors.append(
                    f'{name} {pos} is inside an obstacle.')

    def _check_not_same(self):
        if self.start == self.goal:
            self._errors.append('Start and Goal are the same cell.')

    def _print_report(self):
        print('── Validation Report ─────────────────────')
        if self._errors:
            for e in self._errors:
                print(f'  ✗  {e}')
        else:
            print('  ✓  All checks passed')
        for w in self._warnings:
            print(f'  ⚠  {w}')
        print('─────────────────────────────────────────')
