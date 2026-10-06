"""
stage2_unknown_map.py  —  Hexapod 2D Path Planning  (Stage 2: Unknown Map)
==========================================================================

STAGE 2: Unknown 2D Map + Frontier Exploration + Reactive Replanning
    1. Start with fully unknown map (-1 everywhere)
    2. Robot explores frontier (boundary of known/unknown)
    3. Uses A* to reach nearest frontier goal
    4. When obstacles block path, uses reactive detour or replans
    5. Incrementally reveals map as robot moves

Run modes
---------
    python stage2_unknown_map.py                 # demo (walls map with unknowns)
    python stage2_unknown_map.py --map random    # random obstacles
    python stage2_unknown_map.py --steps 50      # limit to 50 exploration steps
    python stage2_unknown_map.py --seed 42       # reproducible
    python stage2_unknown_map.py --verbose       # print detailed logs

Dependencies (from Stage 1)
---------------------------
    map_creation/: map_generator, map_inflator, frontier_explorer
    path_finding/: astar_planner, path_smoother, path_converter, reactive_planner

Exploration loop
----------------
    while frontier_exists:
        1. Find frontier cells
        2. Pick nearest frontier goal
        3. Run A* to frontier goal
        4. Smooth path
        5. Follow path waypoint-by-waypoint:
           - Reveal map around current position
           - Check if path is still valid
           - If blocked: try reactive detour → else full replan
           - Continue to next waypoint
        6. Upon reaching frontier: expand knowledge, repeat
"""

import argparse
import os
import sys
import json
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional

# Import Stage 1 modules (unchanged)
from map_creation.map_generator import MapGenerator
from map_creation.map_inflator import MapInflator
from map_creation.map_validator import MapValidator

# Import Stage 2 modules (new)
from map_creation.frontier_explorer import FrontierExplorer
from path_finding.astar_planner import AStarPlanner
from path_finding.path_smoother import PathSmoother
from path_finding.path_converter import PathConverter
from path_finding.reactive_planner import ReactivePlanner, PathMonitor


# ══════════════════════════════════════════════════════════════════════
# Configuration
# ══════════════════════════════════════════════════════════════════════

MAP_WIDTH = 200
MAP_HEIGHT = 200
MAP_RESOLUTION = 0.05

START = (10, 10)

ROBOT_RADIUS_CELLS = 2
SENSOR_RANGE_CELLS = 15  # How far ahead robot can "see"

RDP_EPSILON = 2.0
WAYPOINT_SPACING = 5.0

MAX_EXPLORATION_STEPS = 100
MAX_REPLANS_PER_GOAL = 3


# ══════════════════════════════════════════════════════════════════════
# Main Stage 2 Explorer
# ══════════════════════════════════════════════════════════════════════

class UnknownMapExplorer:
    """
    Orchestrates frontier-based exploration of unknown maps.
    """

    def __init__(self, args):
        self.args = args
        self.verbose = args.verbose

        # Maps
        self.ground_truth_map = None  # The "real" map (for simulation)
        self.known_map = None  # What robot has revealed so far (-1/0/1)
        self.inflated_grid = None  # For collision checking

        # Components
        self.map_gen = MapGenerator(MAP_WIDTH, MAP_HEIGHT)
        self.inflator = MapInflator(ROBOT_RADIUS_CELLS)
        self.explorer = None
        self.planner = None
        self.reactor = None
        self.smoother = PathSmoother()
        self.converter = PathConverter(MAP_RESOLUTION)

        # Exploration state
        self.current_pos = START
        self.goal_pos = None
        self.current_path = []
        self.path_monitor = None

        # Stats
        self.exploration_steps = 0
        self.replans = 0
        self.detours = 0
        self.cells_revealed = 0

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def initialize(self):
        """Set up ground truth map and start exploration."""
        print('\n━━━  STAGE 2: Unknown Map Exploration  ━━━━━━━━━━━━━━━━')

        # Generate ground truth (what really exists)
        if self.args.map == 'random':
            self.ground_truth_map = self.map_gen.random_obstacles(
                num_obstacles=18, seed=self.args.seed)
        elif self.args.map == 'maze':
            self.ground_truth_map = self.map_gen.maze()
        elif self.args.map == 'shapes':
            self.ground_truth_map = self.map_gen.shapes()
        else:
            self.ground_truth_map = self.map_gen.walls()

        # Inflate for collision checking
        self.inflated_grid = self.inflator.inflate(self.ground_truth_map)

        # Initialize known map (everything unknown)
        self.known_map = np.full((MAP_HEIGHT, MAP_WIDTH), -1, dtype=int)

        # Initialize components
        self.explorer = FrontierExplorer(
            self.known_map, self.inflated_grid, ROBOT_RADIUS_CELLS)
        self.reactor = ReactivePlanner(self.inflated_grid, max_detour_cost=20.0)
        self.path_monitor = PathMonitor(self.inflated_grid)

        # Reveal starting area
        self._reveal_around_position(self.current_pos, SENSOR_RANGE_CELLS)

        print(f'  Ground truth map size: {MAP_WIDTH} × {MAP_HEIGHT} cells')
        print(f'  Start position: {self.current_pos}')
        print(f'  Initial reveal radius: {SENSOR_RANGE_CELLS} cells')

    # ------------------------------------------------------------------
    # Main exploration loop
    # ------------------------------------------------------------------

    def explore(self):
        """Run the frontier exploration loop."""
        print('\n━━━  Exploration Loop  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')

        for step in range(MAX_EXPLORATION_STEPS):
            self.exploration_steps = step + 1

            # Check for frontier
            frontier = self.explorer.find_frontier()
            stats = self.explorer.get_frontier_stats()

            print(f'\n  Step {self.exploration_steps}: {len(frontier)} frontier cells')
            print(f'    Unknown: {stats["unknown_percentage"]}%  ' 
                  f'Frontier: {stats["frontier_cells"]}')

            if len(frontier) == 0:
                print('\n  ✓ Exploration complete! No more frontier.')
                break

            # Pick next exploration goal
            goal = self.explorer.pick_nearest_frontier(self.current_pos, frontier)
            if goal is None:
                break

            self.goal_pos = goal

            if self.verbose:
                print(f'    → Goal: {goal}')

            # Plan path to frontier goal
            success = self._plan_and_follow_to_goal()
            if not success:
                print(f'    ✗ Failed to reach goal {goal}')
                break

            # Reveal around new position
            self._reveal_around_position(self.current_pos, SENSOR_RANGE_CELLS)

    # ------------------------------------------------------------------
    # Path planning and following
    # ------------------------------------------------------------------

    def _plan_and_follow_to_goal(self) -> bool:
        """
        Plan a path to goal and follow it, handling obstacles.

        Returns
        -------
        True if goal was reached, False if unreachable or max replans exceeded.
        """
        replans_this_goal = 0

        while replans_this_goal < MAX_REPLANS_PER_GOAL:
            # A* plan to goal
            if self.verbose:
                print(f'    Planning A* to {self.goal_pos}...')

            planner = AStarPlanner(self.inflated_grid, self.current_pos, self.goal_pos)
            result = planner.plan()

            if not result.found:
                print(f'    ✗ A* failed to find path to {self.goal_pos}')
                return False

            # Smooth path
            path = result.path
            path = self.smoother.rdp_simplify(path, epsilon=RDP_EPSILON)
            path = self.smoother.interpolate(path, spacing=WAYPOINT_SPACING)

            self.current_path = path
            self.path_monitor.set_path(path)

            if self.verbose:
                print(f'    Path: {len(path)} waypoints (A*: {len(result.path)})')

            # Follow path waypoint by waypoint
            reached_goal = self._follow_path(path)

            if reached_goal:
                if self.verbose:
                    print(f'    ✓ Reached goal {self.goal_pos}')
                return True
            else:
                replans_this_goal += 1
                self.replans += 1
                if self.verbose:
                    print(f'    Replan {replans_this_goal}/{MAX_REPLANS_PER_GOAL}')

        print(f'    ✗ Max replans ({MAX_REPLANS_PER_GOAL}) exceeded')
        return False

    def _follow_path(self, path: List[Tuple[int, int]]) -> bool:
        """
        Follow a path waypoint-by-waypoint, checking for obstacles.

        Returns
        -------
        True if goal reached, False if path was blocked.
        """
        for i, waypoint in enumerate(path):
            self.current_pos = waypoint

            # Reveal around current position
            self._reveal_around_position(self.current_pos, SENSOR_RANGE_CELLS)

            # Check if path is still valid up to goal
            remaining_path = path[i:]
            blockage = self.explorer.find_path_blockage(remaining_path)

            if blockage is not None:
                if self.verbose:
                    print(f'    ✗ Path blocked at {blockage}')

                # Try reactive detour
                detour = self.reactor.find_detour(path, blockage)

                if detour is not None:
                    detour_cost = self.reactor.estimate_detour_cost(
                        detour,
                        np.sqrt((self.goal_pos[0] - self.current_pos[0])**2 +
                               (self.goal_pos[1] - self.current_pos[1])**2)
                    )

                    if detour_cost < 2.0:  # Accept detour if < 2x longer
                        if self.verbose:
                            print(f'    ↻ Taking detour (cost ratio {detour_cost:.2f})')
                        self.detours += 1
                        # Continue with new path from here
                        return self._follow_path(detour + remaining_path[1:])

                # Detour not viable, need full replan
                return False

            # Check if reached goal
            if waypoint == self.goal_pos:
                return True

        # Reached end of path
        return self.current_pos == self.goal_pos

    # ------------------------------------------------------------------
    # Map revelation (sensor simulation)
    # ------------------------------------------------------------------

    def _reveal_around_position(self, pos: Tuple[int, int], radius: int):
        """
        Simulate sensor revealing map around current position.

        Marks all cells within radius as either free (0) or obstacle (1)
        based on ground truth.
        """
        r, c = pos
        revealed_new = 0

        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                nr, nc = r + dr, c + dc

                if not (0 <= nr < MAP_HEIGHT and 0 <= nc < MAP_WIDTH):
                    continue

                # Already known?
                if self.known_map[nr, nc] != -1:
                    continue

                # Reveal from ground truth
                self.known_map[nr, nc] = self.ground_truth_map[nr, nc]
                revealed_new += 1

        self.cells_revealed += revealed_new

        if self.verbose and revealed_new > 0:
            print(f'      Revealed {revealed_new} cells (total: {self.cells_revealed})')

    # ------------------------------------------------------------------
    # Visualization and output
    # ------------------------------------------------------------------

    def visualize_final_state(self):
        """Show final maps side-by-side."""
        fig, axes = plt.subplots(1, 3, figsize=(18, 6))

        # Ground truth
        axes[0].imshow(self.ground_truth_map, cmap='gray', origin='upper')
        axes[0].plot(*self.current_pos[::-1], 'r*', markersize=15, label='Final pos')
        axes[0].set_title('Ground Truth Map')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        # Known map (revealed)
        known_vis = self.known_map.copy()
        known_vis[known_vis == -1] = 0.5  # Show unknowns as gray
        axes[1].imshow(known_vis, cmap='RdYlGn', origin='upper', vmin=-1, vmax=1)
        axes[1].plot(*self.current_pos[::-1], 'r*', markersize=15, label='Final pos')
        axes[1].plot(*START[::-1], 'b*', markersize=15, label='Start')
        axes[1].set_title(f'Known Map (Revealed {self.cells_revealed} cells)')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        # Coverage map
        coverage = (self.known_map != -1).astype(float)
        axes[2].imshow(coverage, cmap='Blues', origin='upper')
        axes[2].set_title(f'Coverage: {100 * np.sum(coverage) / coverage.size:.1f}%')
        axes[2].grid(True, alpha=0.3)

        plt.tight_layout()
        if self.args.save:
            plt.savefig('outputs/stage2_exploration_result.png', dpi=100)
            print('\n  Saved visualization → outputs/stage2_exploration_result.png')
        plt.show()

    def print_summary(self):
        """Print exploration statistics."""
        coverage_pct = 100 * self.cells_revealed / (MAP_WIDTH * MAP_HEIGHT)

        print('\n━━━  Exploration Summary  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        print(f'  Exploration steps : {self.exploration_steps}')
        print(f'  Cells revealed    : {self.cells_revealed} / {MAP_WIDTH * MAP_HEIGHT}  ({coverage_pct:.1f}%)')
        print(f'  Final position    : {self.current_pos}')
        print(f'  Replans executed  : {self.replans}')
        print(f'  Reactive detours  : {self.detours}')
        print()


# ══════════════════════════════════════════════════════════════════════
# CLI and entry point
# ══════════════════════════════════════════════════════════════════════

def parse_args():
    p = argparse.ArgumentParser(
        description='Hexapod 2D path planning — Stage 2 (unknown map + frontier)')
    p.add_argument('--map', default='walls',
                   choices=['walls', 'maze', 'random', 'shapes'],
                   help='Ground truth map type')
    p.add_argument('--steps', type=int, default=MAX_EXPLORATION_STEPS,
                   help=f'Max exploration steps (default: {MAX_EXPLORATION_STEPS})')
    p.add_argument('--seed', type=int, default=42,
                   help='Random seed for reproducibility')
    p.add_argument('--save', action='store_true',
                   help='Save outputs to ./outputs/')
    p.add_argument('--verbose', '-v', action='store_true',
                   help='Print detailed logs')
    return p.parse_args()


def run(args):
    os.makedirs('outputs', exist_ok=True)

    explorer = UnknownMapExplorer(args)
    explorer.initialize()
    explorer.explore()
    explorer.print_summary()

    if not args.verbose or input('\nVisualize? (y/n) ').lower() == 'y':
        explorer.visualize_final_state()


if __name__ == '__main__':
    run(parse_args())
