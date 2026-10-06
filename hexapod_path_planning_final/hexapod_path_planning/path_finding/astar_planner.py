"""
astar_planner.py
================
A* path planner for a 2D occupancy grid.

Algorithm recap
---------------
A* finds the shortest path from start to goal by maintaining:
  • open set  — cells to explore, ordered by f = g + h
  • closed set — cells already expanded

At each step it pops the cell with the lowest f score, expands its
neighbours, and records the cheapest route found so far (g score).

Cost model (8-directional movement)
------------------------------------
  Cardinal move  (N/S/E/W)   : cost = 1.0
  Diagonal move  (NE/SW/…)   : cost = √2 ≈ 1.414

Heuristic
---------
  Euclidean distance to goal — admissible (never over-estimates),
  so A* is guaranteed to find the optimal path.
"""

import numpy as np
from heapq import heappush, heappop
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict


# 8-directional movement: (row_delta, col_delta, cost)
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


@dataclass(order=True)
class _Node:
    """Priority queue node for A*."""
    f: float
    # tie-break counter so dataclass comparison never reaches non-comparable fields
    count: int
    g: float        = field(compare=False)
    row: int        = field(compare=False)
    col: int        = field(compare=False)
    parent: object  = field(compare=False, default=None)  # _Node | None


class AStarPlanner:
    """
    A* planner on a 2D occupancy grid.

    Parameters
    ----------
    grid : 2D numpy array
        0 = free space, 1 = obstacle (or inflated obstacle).
    start : (row, col)
        Start position in grid coordinates.
    goal : (row, col)
        Goal position in grid coordinates.

    Usage
    -----
    planner = AStarPlanner(grid, start=(10, 10), goal=(180, 180))
    result  = planner.plan()

    if result.found:
        print(f'Path: {len(result.path)} waypoints')
        print(f'Cost: {result.cost:.2f}')
        print(f'Nodes explored: {result.explored_count}')
        planner.visualize(result)
    """

    def __init__(
        self,
        grid: np.ndarray,
        start: Tuple[int, int],
        goal: Tuple[int, int],
    ):
        self.grid  = grid
        self.start = start
        self.goal  = goal
        self.height, self.width = grid.shape

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def plan(self) -> 'PlanResult':
        """
        Run A* and return a PlanResult.

        Returns
        -------
        PlanResult with fields:
            found          : bool
            path           : list[(row, col)] — empty if not found
            cost           : float — total path cost
            explored_count : int   — number of cells expanded
            explored       : list[(row, col)] — all expanded cells
        """
        counter   = 0
        open_heap : List[_Node] = []
        open_dict : Dict[Tuple, _Node] = {}
        closed    : set = set()
        explored  : List[Tuple] = []

        start_node = _Node(
            f=self._h(self.start),
            count=counter,
            g=0.0,
            row=self.start[0],
            col=self.start[1],
        )
        heappush(open_heap, start_node)
        open_dict[self.start] = start_node

        while open_heap:
            current = heappop(open_heap)
            pos = (current.row, current.col)

            # Already processed via a cheaper route
            if pos in closed:
                continue

            closed.add(pos)
            explored.append(pos)

            # ── Goal reached ──────────────────────────────────────────
            if pos == self.goal:
                path = self._reconstruct(current)
                return PlanResult(
                    found=True,
                    path=path,
                    cost=current.g,
                    explored_count=len(explored),
                    explored=explored,
                )

            # ── Expand neighbours ─────────────────────────────────────
            for dr, dc, move_cost in DIRECTIONS:
                nr, nc = current.row + dr, current.col + dc
                npos   = (nr, nc)

                # Bounds check
                if not (0 <= nr < self.height and 0 <= nc < self.width):
                    continue

                # Obstacle check
                if self.grid[nr, nc] == 1:
                    continue

                # Already expanded
                if npos in closed:
                    continue

                new_g = current.g + move_cost
                new_f = new_g + self._h(npos)

                # Only add if cheaper than a previously queued route
                if npos in open_dict and open_dict[npos].g <= new_g:
                    continue

                counter += 1
                neighbour = _Node(
                    f=new_f,
                    count=counter,
                    g=new_g,
                    row=nr,
                    col=nc,
                    parent=current,
                )
                heappush(open_heap, neighbour)
                open_dict[npos] = neighbour

        # Open set exhausted — no path exists
        return PlanResult(
            found=False,
            path=[],
            cost=float('inf'),
            explored_count=len(explored),
            explored=explored,
        )

    def visualize(self, result: 'PlanResult', save_path: str = None) -> None:
        """Visualise the grid, explored cells, and found path."""
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches

        fig, ax = plt.subplots(figsize=(10, 10))
        ax.imshow(self.grid, cmap='Greys', origin='upper', vmin=0, vmax=1)

        # Explored cells
        if result.explored:
            exp = np.array(result.explored)
            ax.plot(exp[:, 1], exp[:, 0], 's',
                    color='#90CAF9', markersize=2, alpha=0.4, label='Explored')

        # Path
        if result.found and result.path:
            p = np.array(result.path)
            ax.plot(p[:, 1], p[:, 0], '-', color='#EF9F27',
                    linewidth=2.5, label=f'Path ({len(result.path)} pts)')
            ax.plot(p[:, 1], p[:, 0], 'o', color='#EF9F27',
                    markersize=3.5)

        # Start / Goal
        ax.plot(self.start[1], self.start[0], 'o', color='#378ADD',
                markersize=14, label='Start',
                markeredgecolor='#0C447C', markeredgewidth=2)
        ax.plot(self.goal[1],  self.goal[0],  '*', color='#639922',
                markersize=20, label='Goal',
                markeredgecolor='#27500A', markeredgewidth=2)

        status = (f'Path found  |  {len(result.path)} waypoints  |  '
                  f'cost {result.cost:.1f}  |  '
                  f'{result.explored_count} cells explored'
                  if result.found else 'No path found')
        ax.set_title(f'A* Path Planning\n{status}', fontsize=13)
        ax.set_xlabel('X (col)')
        ax.set_ylabel('Y (row)')
        ax.legend(fontsize=11)
        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f'  Saved → {save_path}')

        plt.show()
        plt.close()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _h(self, pos: Tuple[int, int]) -> float:
        """Euclidean distance heuristic."""
        return np.sqrt(
            (pos[0] - self.goal[0]) ** 2 +
            (pos[1] - self.goal[1]) ** 2
        )

    @staticmethod
    def _reconstruct(node: _Node) -> List[Tuple[int, int]]:
        """Walk parent pointers from goal back to start, then reverse."""
        path = []
        while node is not None:
            path.append((node.row, node.col))
            node = node.parent
        return path[::-1]


# ------------------------------------------------------------------
# Result container
# ------------------------------------------------------------------

@dataclass
class PlanResult:
    """Returned by AStarPlanner.plan()."""
    found          : bool
    path           : List[Tuple[int, int]]
    cost           : float
    explored_count : int
    explored       : List[Tuple[int, int]]

    def __str__(self):
        if self.found:
            return (f'PlanResult(found=True, waypoints={len(self.path)}, '
                    f'cost={self.cost:.2f}, explored={self.explored_count})')
        return f'PlanResult(found=False, explored={self.explored_count})'
