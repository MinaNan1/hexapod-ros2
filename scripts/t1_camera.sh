#!/bin/bash
# TERMINAL — Astra Pro (Orbbec) depth camera bring-up. Dual-driver pattern.
#
# The original Astra Pro has two USB interfaces:
#   - 2bc5:0501 OpenNI depth   → driven by orbbec_camera (OrbbecSDK_ROS2 main)
#   - 2bc5:0403 UVC color      → driven by v4l2_camera   (/dev/video0)
# OrbbecSDK_ROS2 cannot route color through UVC, so we run two ROS2 nodes
# side by side. Both publish under /camera/* and downstream nodes (RTAB-Map,
# YOLO, the web UI) don't notice they're from different drivers.
#
# Static TF base_link → camera_link:
#   x = +0.10 m, y = 0, z = +0.08 m, no rotation.
#   MEASURE THE ACTUAL OFFSET on the assembled robot and edit these args.

# Kill any stale camera nodes from a previous launch
pkill -f orbbec_camera_node           2>/dev/null || true
pkill -f component_container          2>/dev/null || true
pkill -f v4l2_camera_node             2>/dev/null || true
pkill -f "static_transform_publisher.*camera_link"             2>/dev/null || true
pkill -f "static_transform_publisher.*camera_color_optical"    2>/dev/null || true
sleep 0.5

source /opt/ros/humble/setup.bash
source /hexapodd/hexapod_ws/install/setup.bash
source /hexapodd/ros2_ws/install/local_setup.bash

# Static TF: base_link → camera_link.  (RTAB-Map needs this in Phase 2.)
ros2 run tf2_ros static_transform_publisher \
    --x 0.10 --y 0.0 --z 0.08 \
    --roll 0 --pitch 0 --yaw 0 \
    --frame-id base_link --child-frame-id camera_link &

# Static TF: camera_link → camera_color_optical_frame.
# v4l2_camera_node stamps color frames with camera_color_optical_frame, but it
# doesn't publish a TF for it. Without this static publisher RTAB-Map rejects
# every RGB-D pair with "frame ... does not exist". The Astra Pro's color and
# IR/depth sensors sit ~25 mm apart along x; we approximate that here. Optical
# frames use image conventions (z forward, x right, y down), same as orbbec's
# depth optical frame, so rotation is identity relative to camera_link.
ros2 run tf2_ros static_transform_publisher \
    --x 0.025 --y 0.0 --z 0.0 \
    --roll 0 --pitch 0 --yaw 0 \
    --frame-id camera_link --child-frame-id camera_color_optical_frame &

sleep 0.3

CAMERA_LOG=/tmp/camera_launch.log

# ── Color: v4l2_camera on the Astra Pro's UVC interface ──────────────────
# Auto-detect which /dev/video* the UVC color belongs to — replugging the
# USB cable into a different port shifts the device number. We pick the
# first /dev/video* whose v4l2-ctl reports "Astra Pro HD Camera".
VIDEO_DEV=""
for d in /dev/video*; do
    [ -e "$d" ] || continue
    if v4l2-ctl -d "$d" --info 2>/dev/null | grep -q "Astra Pro"; then
        VIDEO_DEV="$d"
        break
    fi
done
if [ -z "$VIDEO_DEV" ]; then
    # Fallback: first /dev/video* we can see (helpful if v4l2-ctl unavailable)
    VIDEO_DEV=$(ls /dev/video* 2>/dev/null | head -1)
fi
echo "[t1_camera.sh] UVC color device: ${VIDEO_DEV:-NONE FOUND}"

# Remap default topics (/image_raw, /camera_info) into the /camera/color/*
# namespace by setting __ns. The web UI subscribes to
# /camera/color/image_raw/compressed (auto-published by image_transport).
ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -r __ns:=/camera/color \
    -p video_device:="$VIDEO_DEV" \
    -p pixel_format:=YUYV \
    -p image_size:=[640,480] \
    -p output_encoding:=rgb8 \
    -p camera_frame_id:=camera_color_optical_frame \
    -p camera_info_url:=file:///hexapodd/config/astra_color_640x480.yaml \
    > "${CAMERA_LOG}.color" 2>&1 &

V4L2_PID=$!
sleep 1

# ── Depth: orbbec_camera, COLOR DISABLED ────────────────────────────────
# Tried astra.launch.py and astra_pro_plus.launch.py with enable_color:=true —
# both fail with "OB_SENSOR_COLOR Match openni video mode failed!" because
# the original Astra Pro has no color on its OpenNI interface. enable_color
# off → depth + IR start cleanly.
(
    ros2 launch orbbec_camera astra_pro_plus.launch.py \
        enable_color:=false \
        enable_depth:=true \
        depth_width:=640 depth_height:=480 depth_fps:=30 depth_format:=Y11 \
        enable_ir:=false \
        enable_point_cloud:=true \
        depth_registration:=false \
        connection_delay:=3000 \
        publish_tf:=true
) 2>&1 | tee "${CAMERA_LOG}.depth"

# Keep the tmux window alive after the launch exits.
echo ""
echo "────────────────────────────────────────────────────────────────"
echo "  Camera launch exited."
echo "  Color log: ${CAMERA_LOG}.color"
echo "  Depth log: ${CAMERA_LOG}.depth"
echo "  This window stays open — Ctrl-b & to kill it from tmux."
echo "────────────────────────────────────────────────────────────────"
# CRITICAL: launch_all runs this via `docker exec` (no TTY), so `read -r`
# would hit EOF instantly and the camera window would die on every launch —
# killing depth, which kills RTAB-Map's map, which kills autonomy. Keep the
# window (and the background v4l2 color node) alive with sleep instead.
sleep infinity

# Cleanup (only reached if the sleep is interrupted)
kill "$V4L2_PID" 2>/dev/null || true
