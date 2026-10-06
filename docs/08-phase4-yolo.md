# Phase 4 — YOLO object detection (spec, not yet implemented)

## Goal

When the hexapod sees a person (or any COCO class), draw a bounding box on
the live camera feed in the web UI, and drop a colored marker at the
estimated 3D position in the Foxglove 3D scene.

## Why this is non-trivial on a Jetson Nano

The Nano has 128 CUDA cores. PyTorch + YOLOv8n CPU runs at ~1 fps; with
TensorRT-quantized weights it jumps to ~10 fps. CUDA is the difference
between "useful real-time detection" and "slideshow."

**The trap**: the current docker container does NOT expose the Jetson's
CUDA runtime. `docker run` in `start_ros.sh` has no `--runtime=nvidia` or
`--gpus all`. So inference inside the container = CPU-only = slow.

There are two ways to fix this; both have trade-offs:

### Option B1 — expose CUDA to the existing container

Add `--runtime=nvidia` to `docker run` in `start_ros.sh`. Requires:
1. `nvidia-container-toolkit` installed on the Jetson host
2. JetPack runtime libraries baked into the docker image (significant
   changes to `hexapod.dockerfile` — switch base image from `ros:humble` to
   `dustynv/ros:humble-llm-r36.4.0` or similar Jetson-friendly base)
3. About 2 GB of new image content

### Option B2 — run inference outside the container, on the host

The Jetson host already has JetPack + CUDA + jetson-inference. We write a
small Python script on the host that subscribes to `/camera/color/image_raw`
via ROS2-on-host (the host has `ros-humble-rclpy`) or via rosbridge (more
portable), runs jetson-inference, and publishes detections back via
WebSocket/rosbridge.

**Recommended**: **B2**. Less invasive to the container, uses the
already-working JetPack stack on the host, no image rebuild.

## Pipeline (B2 design)

```
Camera (/camera/color/image_raw)
        ↓ rosbridge ws://localhost:9090
host-side yolo_node.py
  ├─ libuvc subscribes to compressed image
  ├─ jetson_inference.detectNet runs SSD-MobileNet-v2 (faster than YOLO on Nano)
  ├─ Each detection → vision_msgs/Detection2D{Array}
  └─ publishes back via rosbridge to /yolo/detections
        ↓
Web UI subscribes, overlays bounding boxes on the camera <img>
Foxglove subscribes, drops markers via /detection_markers (visualization_msgs/MarkerArray)
```

## Files to add

| File | Purpose |
|---|---|
| `hexapod_host/yolo_node.py` | Host-side detector. Uses `jetson_inference` + `roslibpy`. Subscribes to camera, publishes detections + markers. |
| `hexapod_host/setup_jetson_inference.sh` | One-time install of `jetson-inference` + `roslibpy` on the Jetson host (NOT container). |
| `scripts/t1_yolo.sh` | Wrapper that activates the venv and runs `yolo_node.py` (host side, not container). |
| `scripts/launch_all.sh` | Add `python3 yolo_node.py` window. |
| `ros2_ws/web_ui/static/js/yolo_overlay.js` | Subscribes to `/yolo/detections`, draws bounding boxes on the camera canvas. |
| `ros2_ws/web_ui/templates/index.html` | New `<canvas id="yolo-overlay">` layered over the camera `<img>`. |

## Test plan

1. Pre-Phase-4: confirm `jetson-inference` runs on the host with a sample image
   (`detectnet images/peds_0.jpg`)
2. Phase 4 step 1: `yolo_node.py` publishes detections when fed a static image
3. Phase 4 step 2: live camera → live detections (verify `/yolo/detections`
   ticks at >2 Hz)
4. Phase 4 step 3: web UI overlay shows bounding boxes
5. Phase 4 step 4: Foxglove shows markers at correct 3D positions

## 3D back-projection

For the Foxglove marker, take the bounding-box centroid `(u, v)` in image
coords, look up the depth value at `(u, v)` in `/camera/depth/image_raw`,
convert to a 3D point in `camera_link` frame using the camera intrinsics
from `/camera/color/camera_info`, then transform to `map` frame using
the live TF. Standard `image_geometry::PinholeCameraModel::projectPixelTo3dRay`
+ `tf2::doTransform`.

## Defer this until

The autonomy stack (Phase 3) is stable enough to actually move the robot
around the room. Without movement, the detector sees the same frame
repeatedly — interesting demo but not actually useful.
