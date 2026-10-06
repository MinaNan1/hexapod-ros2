"""
main.py  —  Hexapod 2D Path Planning  (Standalone / No ROS2 required)
======================================================================
This is the single entry point that wires together every module:

    map_creation/
        map_generator.py   → build / load 2D occupancy grids
        map_inflator.py    → expand obstacles for robot body radius
        map_validator.py   → sanity-check map and positions

    path_finding/
        astar_planner.py   → find optimal path with A*
        path_smoother.py   → remove redundant waypoints (RDP)
        path_converter.py  → grid coords → world coords → body-pose commands

Run modes
---------
    python main.py                      # default demo (walls map)
    python main.py --map random         # random obstacles
    python main.py --map maze
    python main.py --map shapes
    python main.py --load my_map.npy    # load saved map
    python main.py --save               # save outputs to disk
    python main.py --no-viz             # skip matplotlib windows

All outputs are saved to ./outputs/  when --save is used.

Next steps (in order)
---------------------
    Stage 1 (here):  2D known map  + A*                ← YOU ARE HERE
    Stage 2 (next):  2D unknown map + frontier explore  (coming soon)
    Stage 3 (later): 3D map + 3D A* / RRT*              (coming later)
"""

import argparse
import os
import sys
import numpy as np

# ── Module imports ─────────────────────────────────────────────────────
from map_creation.map_generator  import MapGenerator
from map_creation.map_inflator   import MapInflator
from map_creation.map_validator  import MapValidator
from path_finding.astar_planner  import AStarPlanner
from path_finding.path_smoother  import PathSmoother
from path_finding.path_converter import PathConverter


# ══════════════════════════════════════════════════════════════════════
# Configuration  (edit these if you are not using CLI flags)
# ══════════════════════════════════════════════════════════════════════

MAP_WIDTH  = 200        # grid cells
MAP_HEIGHT = 200        # grid cells
MAP_RESOLUTION = 0.05   # metres per cell  (200 cells × 0.05 = 10 m square)

START = (10, 10)        # (row, col)  — top-left area
GOAL  = (185, 185)      # (row, col)  — bottom-right area

ROBOT_RADIUS_CELLS = 2  # inflate obstacles by this many cells
                        # rule: body_radius_m / MAP_RESOLUTION
                        # e.g. 0.10 m / 0.05 = 2 cells

RDP_EPSILON        = 2.0   # path simplification tolerance (cells)
WAYPOINT_SPACING   = 5.0   # interpolation spacing after RDP (cells)


# ══════════════════════════════════════════════════════════════════════
# Main pipeline
# ══════════════════════════════════════════════════════════════════════

def run(args):
    os.makedirs('outputs', exist_ok=True)

    # ──────────────────────────────────────────────────────────────────
    # STEP 1 — Generate or load map
    # ──────────────────────────────────────────────────────────────────
    print('\n━━━  STEP 1: Map Creation  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')

    gen = MapGenerator(width=MAP_WIDTH, height=MAP_HEIGHT)

    if args.load:
        print(f'  Loading map from {args.load}')
        raw_grid = MapGenerator.load(args.load)
    elif args.map == 'random':
        print('  Generating random obstacle map …')
        raw_grid = gen.random_obstacles(num_obstacles=18, seed=42)
    elif args.map == 'maze':
        print('  Generating maze map …')
        raw_grid = gen.maze()
    elif args.map == 'shapes':
        print('  Generating shapes map …')
        raw_grid = gen.shapes()
    else:
        print('  Generating walls map (default) …')
        raw_grid = gen.walls()

    print(f'  Map size: {raw_grid.shape[1]} × {raw_grid.shape[0]} cells  '
          f'({raw_grid.shape[1] * MAP_RESOLUTION:.1f} m × '
          f'{raw_grid.shape[0] * MAP_RESOLUTION:.1f} m)')
    print(f'  Obstacle density: '
          f'{100 * np.sum(raw_grid) / raw_grid.size:.1f}%')

    if args.save:
        gen.save(raw_grid, 'outputs/my_map')

    # ──────────────────────────────────────────────────────────────────
    # STEP 2 — Inflate obstacles for robot body radius
    # ──────────────────────────────────────────────────────────────────
    print('\n━━━  STEP 2: Obstacle Inflation (C-Space)  ━━━━━━━━━━━━━━━━')

    inflator     = MapInflator(robot_radius_cells=ROBOT_RADIUS_CELLS)
    inflated_grid = inflator.inflate(raw_grid)
    stats        = inflator.stats(raw_grid, inflated_grid)

    print(f'  Robot radius : {ROBOT_RADIUS_CELLS} cells  '
          f'≈ {ROBOT_RADIUS_CELLS * MAP_RESOLUTION * 1000:.0f} mm')
    print(f'  Obstacles before inflation : {stats["original_obstacles"]} cells')
    print(f'  Obstacles after  inflation : {stats["inflated_obstacles"]} cells  '
          f'(+{stats["added_cells"]})')
    print(f'  Free space reduction       : {stats["free_reduction_%"]}%')

    if not args.no_viz:
        inflator.compare(raw_grid, inflated_grid,
                         save_path='outputs/inflation_comparison.png' if args.save else None)

    # ──────────────────────────────────────────────────────────────────
    # STEP 3 — Validate map + positions
    # ──────────────────────────────────────────────────────────────────
    print('\n━━━  STEP 3: Validation  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')

    validator = MapValidator(inflated_grid, START, GOAL)
    validator.summary()

    if not validator.validate():
        print('\n  Cannot plan — fix validation errors above.')
        sys.exit(1)

    # ──────────────────────────────────────────────────────────────────
    # STEP 4 — Run A*
    # ──────────────────────────────────────────────────────────────────
    print('\n━━━  STEP 4: A* Path Planning  ━━━━━━━━━━━━━━━━━━━━━━━━━━━')
    print(f'  Start : row={START[0]}, col={START[1]}')
    print(f'  Goal  : row={GOAL[0]},  col={GOAL[1]}')
    print('  Running A* …', end=' ', flush=True)

    planner = AStarPlanner(inflated_grid, START, GOAL)
    result  = planner.plan()

    if not result.found:
        print('FAILED')
        print('\n  No path found. Possible causes:')
        print('    • Goal is surrounded by inflated obstacles.')
        print('    • Map has no free corridor between start and goal.')
        print('    • Increase map size or reduce robot radius.')
        sys.exit(1)

    print('DONE')
    print(f'  Raw waypoints   : {len(result.path)}')
    print(f'  Total cost      : {result.cost:.2f} cells')
    print(f'  Cells explored  : {result.explored_count}')

    if not args.no_viz:
        gen.visualize(
            raw_grid, START, GOAL,
            path=result.path,
            title=f'A* Path  ({len(result.path)} waypoints)',
            save_path='outputs/astar_path.png' if args.save else None,
        )

    # ──────────────────────────────────────────────────────────────────
    # STEP 5 — Smooth path
    # ──────────────────────────────────────────────────────────────────
    print('\n━━━  STEP 5: Path Smoothing (RDP)  ━━━━━━━━━━━━━━━━━━━━━━━')

    smoother     = PathSmoother()
    rdp_path     = smoother.rdp_simplify(result.path, epsilon=RDP_EPSILON)
    final_path   = smoother.interpolate(rdp_path,     spacing=WAYPOINT_SPACING)

    smoother.stats(result.path, final_path)

    if not args.no_viz:
        gen.visualize(
            raw_grid, START, GOAL,
            path=final_path,
            title=f'Smoothed Path  ({len(final_path)} waypoints)',
            save_path='outputs/smooth_path.png' if args.save else None,
        )

    # ──────────────────────────────────────────────────────────────────
    # STEP 6 — Convert to world coords + body-pose commands
    # ──────────────────────────────────────────────────────────────────
    print('\n━━━  STEP 6: Path → Body-Pose Commands  ━━━━━━━━━━━━━━━━━━')

    converter    = PathConverter(map_resolution=MAP_RESOLUTION)
    world_path   = converter.grid_to_world(final_path)
    commands     = converter.path_to_commands(world_path)

    print(f'  World path spans: '
          f'x=[{min(p[0] for p in world_path):.2f}, '
          f'{max(p[0] for p in world_path):.2f}] m  '
          f'y=[{min(p[1] for p in world_path):.2f}, '
          f'{max(p[1] for p in world_path):.2f}] m')
    print(f'  Total segments  : {len(commands)}')
    print()
    converter.print_commands(commands)

    if args.save:
        import json
        with open('outputs/commands.json', 'w') as f:
            json.dump(commands, f, indent=2)
        print('\n  Saved body-pose commands → outputs/commands.json')

    # ──────────────────────────────────────────────────────────────────
    # Summary
    # ──────────────────────────────────────────────────────────────────
    print('\n━━━  Done  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
    print(f'  Map size        : {MAP_WIDTH} × {MAP_HEIGHT} cells')
    print(f'  Robot radius    : {ROBOT_RADIUS_CELLS} cells')
    print(f'  A* path         : {len(result.path)} waypoints  (cost {result.cost:.1f})')
    print(f'  Smoothed path   : {len(final_path)} waypoints')
    print(f'  Motion commands : {len(commands)} segments')
    print()
    print('  Next stage: integrate unknown 2D map with frontier exploration.')
    print()


# ══════════════════════════════════════════════════════════════════════
# CLI
# ══════════════════════════════════════════════════════════════════════

def parse_args():
    p = argparse.ArgumentParser(
        description='Hexapod 2D path planning — Stage 1 (known map + A*)')
    p.add_argument('--map',    default='walls',
                   choices=['walls', 'maze', 'random', 'shapes'],
                   help='Map type to generate (default: walls)')
    p.add_argument('--load',   default=None, metavar='FILE',
                   help='Load existing .npy map instead of generating')
    p.add_argument('--save',   action='store_true',
                   help='Save map, path images, and commands to ./outputs/')
    p.add_argument('--no-viz', action='store_true',
                   help='Skip all matplotlib visualisation windows')
    return p.parse_args()


if __name__ == '__main__':
    run(parse_args())
