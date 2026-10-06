"""
rtabmap.launch.py
=================
Phase 2 of the perception integration. Brings up RTAB-Map in
LIDAR-primary + depth-secondary mode:

    LDS-01 /scan                   → ICP scan-matching, pose correction
    /camera/depth/image_raw        → 3D point-cloud accumulation
    odom (gait dead-reckoning)     → pose prior

Publishes:
    /rtabmap/grid_map (nav_msgs/OccupancyGrid)  → 2D occupancy
    /rtabmap/mapPath  (nav_msgs/Path)           → trajectory
    /rtabmap/cloud_map (sensor_msgs/PointCloud2) → 3D map (depth-only, uncolored)
    /tf:  map → odom   correction
    /tf:  RTAB-Map internal frames

WHY DEPTH-ONLY (NO COLOR FUSION):
The Astra Pro has a UVC color interface (driven by v4l2_camera) that's NOT
registered with the depth sensor. Without manual chessboard calibration of
the depth↔color extrinsics, we can't align them, so we tell RTAB-Map to use
only LIDAR (for pose) and depth (for 3D structure). Color stays available
to the operator UI and to Phase 4 (object detection).

TF conflict note:
display_headless.launch.py publishes a static `world→odom` for development.
RTAB-Map will publish `map→odom`. Two parents for `odom` breaks TF. The
launching script (t2_rtabmap.sh) kills the conflicting publisher first.

Usage:
    ros2 launch hexapod_gazebo rtabmap.launch.py
    # or with custom params:
    ros2 launch hexapod_gazebo rtabmap.launch.py rtabmap_args:="-d"   # delete existing db
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('hexapod_gazebo')

    # Common RTAB-Map / SLAM params shared by every node below
    rtabmap_args = LaunchConfiguration('rtabmap_args')

    common_params = [{
        # ── Frames ────────────────────────────────────────────────
        'frame_id':       'base_link',
        'odom_frame_id':  'odom',
        'map_frame_id':   'map',
        'publish_tf':     True,
        'use_sim_time':   False,

        # ── Input topics ──────────────────────────────────────────
        # LIDAR-ONLY 2D SLAM. The Astra Pro depth stream is unreliable on
        # USB-2 (won't hold a stream), and with subscribe_depth=True RTAB-Map
        # waits for synced depth+scan and produces NO map when depth drops —
        # which killed the 2D map AND autonomy. The LDS-01 is reliable, so we
        # map from /scan alone. This makes /rtabmap/map + map→odom TF + the
        # autonomy pipeline work without the camera.
        # ⚠ Trade-off: no 3D point cloud (/rtabmap/cloud_map stays empty) until
        # depth is restored. To re-enable 3D once the camera is on a POWERED
        # USB hub: set subscribe_depth=True and Grid/FromDepth='true' below.
        'subscribe_depth':  True,     # depth fixed (640x480@30) → 3D cloud back on
        'subscribe_rgb':    False,
        'subscribe_scan':   True,     # LDS-01 /scan still the pose anchor (ICP)
        'subscribe_rgbd':   False,
        'approx_sync':      True,     # depth + scan from different drivers
        'queue_size':       10,
                                  # on the Nano. 10 is plenty for ~15 fps inputs.
        'wait_for_transform_duration': 0.5,

        # ── Registration / pose estimation ────────────────────────
        # 1 = visual+ICP, 2 = ICP only.  LIDAR-anchored map for a moving
        # hexapod with crude gait odom — ICP-only is the robust choice.
        'Reg/Strategy':            '2',
        'Reg/Force3DoF':           'true',   # robot moves on a plane
        'Icp/PointToPlane':        'false',  # 2D scans
        'Icp/VoxelSize':           '0.05',
        'Icp/MaxCorrespondenceDistance': '0.1',
        'Icp/Iterations':          '30',
        'RGBD/NeighborLinkRefining': 'true',

        # ── Occupancy grid ────────────────────────────────────────
        # Build the 2D occupancy grid from the LIDAR scan (not depth), since
        # depth is disabled above. Set back to 'true' when re-enabling depth.
        'Grid/FromDepth':          'true',     # build 3D cloud from depth (walls!)
        'Grid/DepthDecimation':    '2',      # 1/4 the depth points. Was 1 (full
                                             # res) — too heavy on the Nano and a
                                             # major cause of CPU starvation →
                                             # motor jitter. 2 keeps a dense cloud.
                                             # (Nano RAM permitting; raise to 2 if it lags)
        'Grid/RangeMax':           '4.0',    # LDS-01 spec is 3.5 m; +0.5 safety
        'Grid/CellSize':           '0.05',
        'Grid/3D':                 'true',
        # MaxObstacleHeight raised 0.4 → 2.0 so the 3D cloud captures FULL walls
        # for a room-like 3D map (the reference look), not just the bottom 40 cm.
        # Side effect: the 2D grid now marks tall objects as obstacles too — fine,
        # walls are obstacles at any height for a walking robot.
        'Grid/MaxObstacleHeight':  '2.0',
        'Grid/MaxGroundHeight':    '0.05',
        'Grid/NoiseFilteringRadius': '0.05',
        'Grid/NoiseFilteringMinNeighbors': '5',
        'RGBD/CreateOccupancyGrid': 'true',
        # ── Depth-obstacle layer reliability (off-plane obstacle avoidance) ──
        # RayTracing: carve out free space along each ray so the grid CLEARS
        # cells the sensor now sees as empty. Without it, depth obstacles only
        # ever get ADDED and never removed → phantom/stale obstacles pile up and
        # the planner boxes the robot in. Essential for moving-robot avoidance.
        'Grid/RayTracing':         'true',
        # Ignore returns closer than 0.5 m so the robot's OWN sprawled legs /
        # body (in the bottom of the camera's forward FOV) aren't mapped as
        # obstacles right in front of it (which would make it think it's stuck).
        'Grid/RangeMin':           '0.5',
        'cloud_min_depth':         '0.5',
        # Cloud assembly knobs — keep memory + bandwidth reasonable on Nano
        'cloud_max_depth':         '4.0',    # ignore depth pts beyond 4 m
        'cloud_voxel_size':        '0.03',   # 3 cm voxels — denser walls than
                                             # 4 cm, WITHOUT the decimation=1 CPU
                                             # hit that caused motor jitter
                                             # (decimation stays 2). Raise toward
                                             # 0.05 if the Nano lags / WiFi can't
                                             # keep up; 0.02 for max detail if idle.
        'cloud_noise_filtering_radius': '0.05',
        'cloud_noise_filtering_min_neighbors': '5',

        # ── Memory / loop closure ─────────────────────────────────
        'Mem/IncrementalMemory':   'true',
        'Mem/InitWMWithAllNodes':  'false',
        'RGBD/OptimizeFromGraphEnd': 'false',

        # ── Keyframe thresholds ───────────────────────────────────
        # Defaults are 0.1 m / 0.1 rad — too coarse for a slow-walking
        # hexapod whose gait odometry under-counts travel distance. Lower
        # them so RTAB-Map adds keyframes (and publishes a fresh grid_map)
        # after just a few steps.
        'RGBD/LinearUpdate':       '0.02',  # was 0.1
        'RGBD/AngularUpdate':      '0.05',  # was 0.1
        # Also tell RTAB-Map to refresh the grid map even if no new
        # keyframe was added — guarantees periodic grid publishes so the
        # browser doesn't sit on a stale cached map forever.
        'Rtabmap/PublishStats':    'true',
    }]

    return LaunchDescription([
        DeclareLaunchArgument(
            'rtabmap_args',
            default_value='',
            description="Extra CLI args to rtabmap_slam (e.g. '-d' to clear db).",
        ),

        # ── rtabmap_slam: the main SLAM node ─────────────────────
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace='rtabmap',
            output='screen',
            parameters=common_params,
            arguments=[rtabmap_args],
            remappings=[
                ('scan',           '/scan'),
                ('rgb/image',      '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image',    '/camera/depth/image_raw'),
                ('depth/camera_info', '/camera/depth/camera_info'),
                ('odom',           '/odom'),
            ],
        ),
    ])
