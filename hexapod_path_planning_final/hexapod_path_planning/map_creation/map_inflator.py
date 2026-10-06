"""
map_inflator.py
===============
Inflates obstacles in a 2D occupancy grid to account for the
hexapod's physical body radius (C-space / configuration-space expansion).

Why this matters
----------------
A* treats the robot as a point.  Your hexapod is NOT a point — it has a
roughly circular body footprint (~150 mm diameter for a typical HX-35H hex).
Inflating obstacles by the body radius guarantees that every path A* finds
is physically traversable: the robot centre can follow the path without the
body clipping any wall.

How it works
------------
For every obstacle cell, all cells within `radius` cells are also marked
as forbidden.  The shape of the inflation kernel can be circular (default)
or square.
"""

import numpy as np
from scipy.ndimage import binary_dilation, generate_binary_structure


class MapInflator:
    """
    Inflates obstacle cells by a given robot body radius.

    Parameters
    ----------
    robot_radius_cells : int
        How many grid cells to expand each obstacle.
        Rule of thumb:  robot_radius_m / map_resolution_m
        Example: body radius = 0.15 m, resolution = 0.05 m/cell → 3 cells

    kernel : 'circle' | 'square'
        Shape of the inflation structuring element.
        'circle' is more accurate for a round body (recommended).
        'square' is faster and slightly more conservative.

    Usage
    -----
    inflator = MapInflator(robot_radius_cells=3)
    inflated_grid = inflator.inflate(raw_grid)

    # Visualise the difference
    inflator.compare(raw_grid, inflated_grid)
    """

    def __init__(self, robot_radius_cells: int = 2, kernel: str = 'circle'):
        if robot_radius_cells < 0:
            raise ValueError('robot_radius_cells must be >= 0')
        self.robot_radius_cells = robot_radius_cells
        self.kernel_type = kernel
        self._kernel = self._build_kernel()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def inflate(self, grid: np.ndarray) -> np.ndarray:
        """
        Return a new grid with obstacles inflated.

        The original grid is NOT modified.

        Parameters
        ----------
        grid : 2D numpy array (0=free, 1=obstacle)

        Returns
        -------
        inflated : 2D numpy array (0=free, 1=obstacle or inflated zone)
        """
        if self.robot_radius_cells == 0:
            return grid.copy()

        obstacle_mask = grid.astype(bool)
        inflated_mask = binary_dilation(obstacle_mask, structure=self._kernel)
        return inflated_mask.astype(np.int8)

    def stats(self, original: np.ndarray, inflated: np.ndarray) -> dict:
        """Return a dict with inflation statistics."""
        total       = original.size
        orig_obs    = int(np.sum(original))
        infl_obs    = int(np.sum(inflated))
        added       = infl_obs - orig_obs
        free_before = total - orig_obs
        free_after  = total - infl_obs

        return {
            'total_cells'     : total,
            'original_obstacles': orig_obs,
            'inflated_obstacles': infl_obs,
            'added_cells'     : added,
            'free_before'     : free_before,
            'free_after'      : free_after,
            'free_reduction_%': round(100 * added / max(free_before, 1), 1),
        }

    def compare(
        self,
        original: np.ndarray,
        inflated: np.ndarray,
        save_path: str = None,
    ) -> None:
        """Side-by-side visualisation of original vs inflated map."""
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 2, figsize=(14, 7))

        axes[0].imshow(original, cmap='Greys', origin='upper', vmin=0, vmax=1)
        axes[0].set_title('Original map', fontsize=13)
        axes[0].set_xlabel('X')
        axes[0].set_ylabel('Y')

        # Build RGB image: obstacle=dark, inflated=pink, free=white
        vis = np.ones((*inflated.shape, 3), dtype=float)  # white base
        vis[inflated == 1] = [0.85, 0.0, 0.0]             # red = combined
        vis[original == 1] = [0.2, 0.2, 0.2]              # dark = true obstacle

        axes[1].imshow(vis, origin='upper')
        axes[1].set_title(
            f'Inflated map (radius={self.robot_radius_cells} cells)\n'
            '■ dark = obstacle   ■ red = inflated zone',
            fontsize=13,
        )
        axes[1].set_xlabel('X')

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f'  Saved comparison → {save_path}')

        plt.show()
        plt.close()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _build_kernel(self) -> np.ndarray:
        """Build the dilation structuring element."""
        r = self.robot_radius_cells
        size = 2 * r + 1

        if self.kernel_type == 'square':
            return np.ones((size, size), dtype=bool)

        # Circular kernel
        y, x = np.ogrid[-r: r + 1, -r: r + 1]
        return (x ** 2 + y ** 2 <= r ** 2)
