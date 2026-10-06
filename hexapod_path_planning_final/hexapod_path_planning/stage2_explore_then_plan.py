"""
stage2_explore_then_plan.py — Explore, Then Plan Optimal Path
==============================================================

SIMPLIFIED TWO-PHASE APPROACH:
    Phase 1: Explore for N steps (discover enough of the map)
    Phase 2: Plan optimal smooth path to goal (like Stage 1)

This is practical and predictable:
    - Explore for fixed number of steps (configurable)
    - Stops exploration at defined step count
    - Then uses Stage 1 planning on discovered map
    - Outputs smooth optimal path

Run modes
---------
    python stage2_explore_then_plan.py                      # default: 30 exploration steps
    python stage2_explore_then_plan.py --goal 100 100       # custom goal
    python stage2_explore_then_plan.py --steps 50           # explore 50 steps
    python stage2_explore_then_plan.py --map random --save  # save visualizations
    python stage2_explore_then_plan.py --verbose            # detailed logs
"""

import argparse
import os
import sys
import json
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional

# Import Stage 1 & 2 modules
from map_creation.map_generator import MapGenerator
from map_creation.map_inflator import MapInflator
from map_creation.frontier_explorer import FrontierExplorer
from path_finding.astar_planner import AStarPlanner
from path_finding.path_smoother import PathSmoother
from path_finding.path_converter import PathConverter


# ══════════════════════════════════════════════════════════════════════
# Configuration
# ══════════════════════════════════════════════════════════════════════

MAP_WIDTH = 200
MAP_HEIGHT = 200
MAP_RESOLUTION = 0.05

START = (10, 10)
DEFAULT_GOAL = (185, 185)

ROBOT_RADIUS_CELLS = 2
SENSOR_RANGE_CELLS = 15

RDP_EPSILON = 2.0
WAYPOINT_SPACING = 5.0

DEFAULT_EXPLORATION_STEPS = 30


# ══════════════════════════════════════════════════════════════════════
# Main System
# ══════════════════════════════════════════════════════════════════════

class ExploreAndPlan:
    """
    Two-phase system:
      Phase 1: Frontier-based exploration for N steps
      Phase 2: Plan optimal smooth path to goal using discovered map
    """

    def __init__(self, args):
        self.args = args
        self.verbose = args.verbose

        # Maps
        self.ground_truth_map = None
        self.known_map = None
        self.inflated_grid = None

        # Components
        self.map_gen = MapGenerator(MAP_WIDTH, MAP_HEIGHT)
        self.inflator = MapInflator(ROBOT_RADIUS_CELLS)
        self.explorer = None
        self.smoother = PathSmoother()
        self.converter = PathConverter(MAP_RESOLUTION)

        # Positions
        self.start_pos = START
        self.goal_pos = tuple(args.goal) if args.goal else DEFAULT_GOAL
        self.current_pos = START

        # Configuration
        self.max_exploration_steps = args.steps

        # Results
        self.exploration_steps = 0
        self.cells_revealed = 0
        self.exploration_path = [START]
        self.final_path = None
        self.final_commands = None

        print(f'\n  Start position : {self.start_pos}')
        print(f'  Goal position  : {self.goal_pos}')
        print(f'  Exploration steps: {self.max_exploration_steps}')

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def initialize(self):
        """Set up ground truth map."""
        print('\n━━━  PHASE 1: Exploration (Discover the Map)  ━━━━━━━━━━━━')

        # Generate ground truth
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

        # Initialize known map (all unknown)
        self.known_map = np.full((MAP_HEIGHT, MAP_WIDTH), -1, dtype=int)

        # Initialize explorer
        self.explorer = FrontierExplorer(
            self.known_map, self.inflated_grid, ROBOT_RADIUS_CELLS)

        # Reveal starting area
        self._reveal_around_position(self.current_pos, SENSOR_RANGE_CELLS)

        print(f'  Map size: {MAP_WIDTH} × {MAP_HEIGHT} cells')
        print(f'  Initial reveal radius: {SENSOR_RANGE_CELLS} cells')

    # ------------------------------------------------------------------
    # Exploration Phase
    # ------------------------------------------------------------------

    def explore_steps(self):
        """Explore frontier for N steps."""

        for step in range(self.max_exploration_steps):
            self.exploration_steps = step + 1

            # Get frontier
            frontier = self.explorer.find_frontier()
            stats = self.explorer.get_frontier_stats()

            print(f'\n  Step {self.exploration_steps}: {len(frontier)} frontier cells')
            print(f'    Unknown: {stats["unknown_percentage"]}%  '
                  f'Revealed: {self.cells_revealed} cells')

            if len(frontier) == 0:
                print(f'  → Frontier exhausted at step {step + 1}')
                break

            # Pick nearest frontier (greedy), skipping any that are
            # already within sensor range — those will be revealed on
            # the next cycle anyway and aren't worth a full A* trip.
            useful_frontier = [
                f for f in frontier
                if abs(f[0] - self.current_pos[0]) + abs(f[1] - self.current_pos[1])
                > SENSOR_RANGE_CELLS
            ] or frontier  # fall back to all frontiers if none are far enough
            goal = self.explorer.pick_nearest_frontier(self.current_pos, useful_frontier)
            if goal is None:
                break

            if self.verbose:
                print(f'    Frontier goal: {goal}')

            # Plan and follow to frontier
            success = self._plan_and_follow_to_frontier(goal)
            if not success:
                if self.verbose:
                    print(f'    Could not reach frontier {goal}')

            # Reveal around new position
            self._reveal_around_position(self.current_pos, SENSOR_RANGE_CELLS)

        coverage = 100 * self.cells_revealed / (MAP_WIDTH * MAP_HEIGHT)
        print(f'\n  ✓ Exploration complete')
        print(f'    Steps: {self.exploration_steps}')
        print(f'    Coverage: {coverage:.1f}%')

    def _plan_and_follow_to_frontier(self, frontier_goal: Tuple[int, int]) -> bool:
        """Plan and follow path to frontier goal."""
        # Plan on known_map (only explored cells) so the robot cannot
        # path through unknown space it hasn't sensed yet.
        # Treat unknown (-1) and obstacles (1) both as impassable.
        known_navigable = np.where(self.known_map == 0, 0, 1).astype(np.int8)
        planner = AStarPlanner(known_navigable, self.current_pos, frontier_goal)
        result = planner.plan()

        if not result.found:
            return False

        # Smooth path
        path = self.smoother.rdp_simplify(result.path, epsilon=RDP_EPSILON)
        path = self.smoother.interpolate(path, spacing=WAYPOINT_SPACING)

        if self.verbose:
            print(f'    Path: {len(path)} waypoints')

        # Follow path
        for waypoint in path:
            self.current_pos = waypoint
            self.exploration_path.append(waypoint)

            # Reveal around position
            self._reveal_around_position(self.current_pos, SENSOR_RANGE_CELLS)

        return True

    # ------------------------------------------------------------------
    # Planning Phase
    # ------------------------------------------------------------------

    def plan_to_goal(self) -> bool:
        """Plan optimal smooth path to goal using discovered map."""
        print('\n━━━  PHASE 2: Path Planning (Optimal Smooth Path)  ━━━━━━━')

        # Check if goal is reachable
        if not self._is_goal_reachable():
            print('  ✗ Goal is not reachable (still in unknown or obstacle area)')
            return False

        # Build a navigable grid from known_map:
        # treat unknown cells (-1) as obstacles so the path only uses
        # area the robot has actually explored.
        known_navigable = np.where(self.known_map == 0, 0, 1).astype(np.int8)

        print(f'  Planning A* from {self.start_pos} to {self.goal_pos}...')

        # A* from start to goal on the discovered navigable area
        planner = AStarPlanner(known_navigable, self.start_pos, self.goal_pos)
        result = planner.plan()

        if not result.found:
            print('  ✗ A* failed — goal not yet connected to start through explored area')
            print('    Try increasing --steps to explore more of the map first')
            return False

        print(f'  Raw A* path: {len(result.path)} waypoints')
        print(f'  Path cost: {result.cost:.2f} cells')

        # Smooth path
        raw_path = result.path
        rdp_path = self.smoother.rdp_simplify(raw_path, epsilon=RDP_EPSILON)
        smooth_path = self.smoother.interpolate(rdp_path, spacing=WAYPOINT_SPACING)

        self.final_path = smooth_path

        print(f'  After RDP: {len(rdp_path)} waypoints')
        print(f'  Final path: {len(smooth_path)} waypoints')

        # Convert to world coords + commands
        world_path = self.converter.grid_to_world(smooth_path)
        commands = self.converter.path_to_commands(world_path)

        self.final_commands = commands

        print(f'\n  World coordinates:')
        print(f'    x: [{min(p[0] for p in world_path):.2f}, '
              f'{max(p[0] for p in world_path):.2f}] m')
        print(f'    y: [{min(p[1] for p in world_path):.2f}, '
              f'{max(p[1] for p in world_path):.2f}] m')
        print(f'  Motion segments: {len(commands)}')

        return True

        if not result.found:
            print('  ✗ A* failed to find path')
            return False

        print(f'  Raw A* path: {len(result.path)} waypoints')
        print(f'  Path cost: {result.cost:.2f} cells')

        # Smooth path
        raw_path = result.path
        rdp_path = self.smoother.rdp_simplify(raw_path, epsilon=RDP_EPSILON)
        smooth_path = self.smoother.interpolate(rdp_path, spacing=WAYPOINT_SPACING)

        self.final_path = smooth_path

        print(f'  After RDP: {len(rdp_path)} waypoints')
        print(f'  Final path: {len(smooth_path)} waypoints')

        # Convert to world coords + commands
        world_path = self.converter.grid_to_world(smooth_path)
        commands = self.converter.path_to_commands(world_path)

        self.final_commands = commands

        print(f'\n  World coordinates:')
        print(f'    x: [{min(p[0] for p in world_path):.2f}, '
              f'{max(p[0] for p in world_path):.2f}] m')
        print(f'    y: [{min(p[1] for p in world_path):.2f}, '
              f'{max(p[1] for p in world_path):.2f}] m')
        print(f'  Motion segments: {len(commands)}')

        return True

    def _is_goal_reachable(self) -> bool:
        """Check if goal position is valid, known, and not in an obstacle zone."""
        r, c = self.goal_pos

        # Check bounds
        if not (0 <= r < MAP_HEIGHT and 0 <= c < MAP_WIDTH):
            print(f'    Goal {self.goal_pos} out of bounds')
            return False

        # Check if goal is known (not unknown)
        if self.known_map[r, c] == -1:
            print(f'    Goal {self.goal_pos} still unknown')
            return False

        # Check inflated_grid — the same map A* uses for final planning.
        # known_map only reflects raw sensor data; inflated_grid expands
        # obstacles by ROBOT_RADIUS_CELLS so a cell that looks free in
        # known_map can still be inside an inflated obstacle zone.
        if self.inflated_grid[r, c] == 1:
            print(f'    Goal {self.goal_pos} is inside inflated obstacle zone')
            return False

        return True

    # ------------------------------------------------------------------
    # Map Revelation
    # ------------------------------------------------------------------

    def _reveal_around_position(self, pos: Tuple[int, int], radius: int):
        """Reveal map around position based on ground truth."""
        r, c = pos
        revealed_new = 0

        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                nr, nc = r + dr, c + dc

                if not (0 <= nr < MAP_HEIGHT and 0 <= nc < MAP_WIDTH):
                    continue

                if self.known_map[nr, nc] != -1:
                    continue

                # Reveal from ground truth
                self.known_map[nr, nc] = self.ground_truth_map[nr, nc]
                revealed_new += 1

        self.cells_revealed += revealed_new

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------

    def visualize_results(self):
        """Show exploration + final path."""
        fig, axes = plt.subplots(1, 3, figsize=(20, 6))

        # Exploration trajectory
        axes[0].imshow(self.ground_truth_map, cmap='gray', origin='upper')
        if len(self.exploration_path) > 1:
            traj = np.array(self.exploration_path)
            axes[0].plot(traj[:, 1], traj[:, 0], 'r-', linewidth=2.5, 
                        label='Exploration path', alpha=0.7)
        axes[0].plot(self.start_pos[1], self.start_pos[0], 'go', markersize=12, label='Start')
        axes[0].plot(self.goal_pos[1], self.goal_pos[0], 'r*', markersize=20, label='Goal')
        axes[0].set_title(f'Exploration Trajectory\n({self.exploration_steps} steps)')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        # Discovered map
        known_vis = self.known_map.copy()
        known_vis[known_vis == -1] = 0.5
        axes[1].imshow(known_vis, cmap='RdYlGn', origin='upper', vmin=-1, vmax=1)
        axes[1].plot(self.start_pos[1], self.start_pos[0], 'go', markersize=12, label='Start')
        axes[1].plot(self.goal_pos[1], self.goal_pos[0], 'r*', markersize=20, label='Goal')
        coverage = 100 * self.cells_revealed / (MAP_WIDTH * MAP_HEIGHT)
        axes[1].set_title(f'Discovered Map\n({coverage:.1f}% coverage)')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        # Final optimal path
        axes[2].imshow(self.ground_truth_map, cmap='gray', origin='upper')
        if self.final_path:
            path_array = np.array(self.final_path)
            axes[2].plot(path_array[:, 1], path_array[:, 0], 'b-', linewidth=3, 
                        label=f'Optimal path ({len(self.final_path)} points)', alpha=0.8)
            axes[2].fill(path_array[:, 1], path_array[:, 0], 'b', alpha=0.1)
        axes[2].plot(self.start_pos[1], self.start_pos[0], 'go', markersize=12, label='Start')
        axes[2].plot(self.goal_pos[1], self.goal_pos[0], 'r*', markersize=20, label='Goal')
        axes[2].set_title(f'Final Smooth Optimal Path\n({len(self.final_path)} waypoints)')
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)

        plt.tight_layout()

        if self.args.save:
            plt.savefig('outputs/explore_and_plan_result.png', dpi=100, bbox_inches='tight')
            print('\n  Saved → outputs/explore_and_plan_result.png')

        plt.show()

    def print_summary(self):
        """Print summary."""
        coverage = 100 * self.cells_revealed / (MAP_WIDTH * MAP_HEIGHT)

        print('\n━━━  SUMMARY  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        print(f'  EXPLORATION PHASE')
        print(f'    Steps: {self.exploration_steps}')
        print(f'    Cells revealed: {self.cells_revealed} / {MAP_WIDTH * MAP_HEIGHT}  ({coverage:.1f}%)')
        print()

        if self.final_path:
            print(f'  PATH PLANNING PHASE')
            print(f'    Optimal path: {len(self.final_path)} waypoints')
            print(f'    Motion commands: {len(self.final_commands)} segments')
            print()
            print(f'  SAMPLE MOTION COMMANDS (first 5):')
            for i, cmd in enumerate(self.final_commands[:5]):
                print(f'    {i+1}. tx={cmd["tx"]:.2f}mm, ty={cmd["ty"]:.2f}mm, '
                      f'yaw={cmd["yaw"]:.3f}rad, hdg={cmd["heading_deg"]:.0f}°')
            if len(self.final_commands) > 5:
                print(f'    ... ({len(self.final_commands) - 5} more segments)')
        else:
            print('  ✗ No path found')

        print('━' * 60)

    def save_outputs(self):
        """Save commands, map, and stats."""
        if not self.final_commands:
            return

        # Save motion commands
        with open('outputs/final_smooth_path_commands.json', 'w') as f:
            json.dump(self.final_commands, f, indent=2)
        print('\n  Saved commands → outputs/final_smooth_path_commands.json')

        # Save discovered map
        np.save('outputs/explored_map.npy', self.known_map)
        print('  Saved discovered map → outputs/explored_map.npy')

        # Save statistics
        stats = {
            'exploration_steps': int(self.exploration_steps),
            'cells_revealed': int(self.cells_revealed),
            'coverage_percent': round(100 * self.cells_revealed / (MAP_WIDTH * MAP_HEIGHT), 1),
            'final_path_waypoints': len(self.final_path) if self.final_path else 0,
            'motion_segments': len(self.final_commands) if self.final_commands else 0,
            'start': self.start_pos,
            'goal': self.goal_pos,
            'map_type': self.args.map,
        }

        with open('outputs/explore_and_plan_stats.json', 'w') as f:
            json.dump(stats, f, indent=2)
        print('  Saved stats → outputs/explore_and_plan_stats.json')


# ══════════════════════════════════════════════════════════════════════
# CLI
# ══════════════════════════════════════════════════════════════════════

def parse_args():
    p = argparse.ArgumentParser(
        description='Explore map for N steps, then plan optimal smooth path to goal')
    p.add_argument('--map', default='walls',
                   choices=['walls', 'maze', 'random', 'shapes'],
                   help='Map type (default: walls)')
    p.add_argument('--goal', type=int, nargs=2, default=None, metavar=('ROW', 'COL'),
                   help=f'Goal position (default: {DEFAULT_GOAL})')
    p.add_argument('--steps', type=int, default=DEFAULT_EXPLORATION_STEPS,
                   help=f'Exploration steps (default: {DEFAULT_EXPLORATION_STEPS})')
    p.add_argument('--seed', type=int, default=42,
                   help='Random seed (default: 42)')
    p.add_argument('--save', action='store_true',
                   help='Save outputs to disk')
    p.add_argument('--verbose', '-v', action='store_true',
                   help='Verbose output')
    return p.parse_args()


def run(args):
    os.makedirs('outputs', exist_ok=True)

    explorer = ExploreAndPlan(args)
    explorer.initialize()

    # Phase 1: Explore
    explorer.explore_steps()

    # Phase 2: Plan
    success = explorer.plan_to_goal()

    if not success:
        print('\n✗ Path planning failed')
        sys.exit(1)

    # Output
    explorer.print_summary()

    if args.save:
        explorer.save_outputs()

    explorer.visualize_results()


if __name__ == '__main__':
    run(parse_args())
