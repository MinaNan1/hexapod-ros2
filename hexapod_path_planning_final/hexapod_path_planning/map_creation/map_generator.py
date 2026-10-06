"""
map_generator.py
================
Generates 2D occupancy grid maps for hexapod path planning.
All maps use:
    0 = free space
    1 = obstacle

Coordinate convention: grid[row, col] → grid[y, x]
"""

import numpy as np
import matplotlib.pyplot as plt
from PIL import Image


class MapGenerator:
    """
    Creates 2D occupancy grids using different methods.

    Usage
    -----
    gen = MapGenerator(width=200, height=200)
    grid = gen.walls()
    grid = gen.random_obstacles(num_obstacles=15, seed=42)
    grid = gen.from_image('my_map.png')
    gen.save(grid, 'my_map')          # saves .npy + .png
    gen.visualize(grid, start, goal)
    """

    def __init__(self, width: int = 200, height: int = 200):
        self.width = width
        self.height = height

    # ------------------------------------------------------------------
    # METHOD 1 — Programmatic obstacles
    # ------------------------------------------------------------------

    def walls(self) -> np.ndarray:
        """Three rectangular walls spread across the map."""
        grid = np.zeros((self.height, self.width), dtype=np.int8)
        grid[40:60,  50:150] = 1   # top horizontal wall
        grid[100:120, 30:100] = 1  # middle-left wall
        grid[140:160, 100:180] = 1 # bottom-right wall
        return grid

    def maze(self) -> np.ndarray:
        """Maze-like corridor pattern."""
        grid = np.zeros((self.height, self.width), dtype=np.int8)
        grid[50:150,  50:70]  = 1
        grid[50:70,  70:150]  = 1
        grid[100:150, 100:120] = 1
        grid[50:100, 140:160] = 1
        return grid

    def random_obstacles(
        self,
        num_obstacles: int = 15,
        min_size: int = 20,
        max_size: int = 50,
        seed: int = None,
    ) -> np.ndarray:
        """
        Randomly placed rectangular obstacles.

        Parameters
        ----------
        num_obstacles : int
            Number of rectangles to place.
        min_size / max_size : int
            Width/height range for each obstacle (pixels).
        seed : int or None
            Random seed for reproducibility.
        """
        if seed is not None:
            np.random.seed(seed)

        grid = np.zeros((self.height, self.width), dtype=np.int8)
        margin = max_size + 5  # keep obstacles away from edges

        for _ in range(num_obstacles):
            x = np.random.randint(10, self.width  - margin)
            y = np.random.randint(10, self.height - margin)
            w = np.random.randint(min_size, max_size)
            h = np.random.randint(min_size, max_size)
            grid[y:y + h, x:x + w] = 1

        return grid

    def shapes(self) -> np.ndarray:
        """Mix of rectangles and circular obstacles."""
        grid = np.zeros((self.height, self.width), dtype=np.int8)

        # Rectangles
        grid[30:60,  40:120] = 1
        grid[120:150, 60:140] = 1

        # Circles using distance mask
        y_idx, x_idx = np.ogrid[:self.height, :self.width]

        # Circle 1 — center (150, 50), radius 25
        grid[(x_idx - 150) ** 2 + (y_idx - 50) ** 2 <= 25 ** 2] = 1

        # Circle 2 — center (80, 150), radius 20
        grid[(x_idx - 80) ** 2 + (y_idx - 150) ** 2 <= 20 ** 2] = 1

        return grid

    # ------------------------------------------------------------------
    # METHOD 2 — Load from image
    # ------------------------------------------------------------------

    def from_image(self, image_path: str, threshold: int = 128) -> np.ndarray:
        """
        Load a PNG/JPG as an occupancy grid.

        Convention: dark pixels (< threshold) become obstacles (1),
                    light pixels become free space (0).

        Parameters
        ----------
        image_path : str
            Path to the image file.
        threshold : int
            Pixel intensity below which a cell is marked as obstacle.
        """
        img = Image.open(image_path).convert('L')
        arr = np.array(img)
        return (arr < threshold).astype(np.int8)

    # ------------------------------------------------------------------
    # SAVE / LOAD
    # ------------------------------------------------------------------

    def save(self, grid: np.ndarray, name: str = 'my_map') -> None:
        """
        Save grid in two formats:
            <name>.npy  — fast binary (use this in Python)
            <name>.png  — human-readable image (white=free, black=obstacle)
        """
        # Binary
        np.save(f'{name}.npy', grid)
        print(f'  Saved {name}.npy')

        # Image  (invert: free=white=255, obstacle=black=0)
        img_array = ((1 - grid) * 255).astype(np.uint8)
        Image.fromarray(img_array).save(f'{name}.png')
        print(f'  Saved {name}.png')

    @staticmethod
    def load(path: str) -> np.ndarray:
        """Load a previously saved .npy map."""
        grid = np.load(path)
        print(f'  Loaded map: {grid.shape[1]}w x {grid.shape[0]}h')
        return grid

    # ------------------------------------------------------------------
    # VISUALIZE
    # ------------------------------------------------------------------

    def visualize(
        self,
        grid: np.ndarray,
        start: tuple = None,
        goal: tuple = None,
        path: list = None,
        title: str = '2D Occupancy Map',
        save_path: str = None,
    ) -> None:
        """
        Display the map with optional start, goal, and path overlay.

        Parameters
        ----------
        grid   : 2D numpy array  (0=free, 1=obstacle)
        start  : (row, col) — plotted as green circle
        goal   : (row, col) — plotted as red star
        path   : list of (row, col) — plotted as blue line
        title  : figure title
        save_path : if set, saves PNG to this path
        """
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.imshow(grid, cmap='Greys', origin='upper', vmin=0, vmax=1)

        if path:
            path_arr = np.array(path)
            ax.plot(path_arr[:, 1], path_arr[:, 0],
                    'b-', linewidth=2, alpha=0.7, label='Path')
            ax.plot(path_arr[:, 1], path_arr[:, 0],
                    'bo', markersize=3, alpha=0.5)

        if start:
            ax.plot(start[1], start[0],
                    'go', markersize=14, label='Start',
                    markeredgecolor='darkgreen', markeredgewidth=2)
        if goal:
            ax.plot(goal[1], goal[0],
                    'r*', markersize=22, label='Goal',
                    markeredgecolor='darkred', markeredgewidth=2)

        ax.set_title(title, fontsize=14)
        ax.set_xlabel('X (col)')
        ax.set_ylabel('Y (row)')
        if start or goal:
            ax.legend(fontsize=12)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f'  Saved visualization → {save_path}')

        plt.show()
        plt.close()
import numpy as np


def create_random_map(size=100, obstacle_prob=0.2):
    """
    Generate a random occupancy grid.

    Returns:
        0 = free
        1 = obstacle
    """
    grid = np.zeros((size, size))
    noise = np.random.rand(size, size)

    grid[noise < obstacle_prob] = 1

    return grid
