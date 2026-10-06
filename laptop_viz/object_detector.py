#!/usr/bin/env python3
"""
object_detector.py — Phase 4 object/person detection, runs on the LAPTOP.

Subscribes to the hexapod's COLOR camera over the ROS network (the compressed
transport, so it's WiFi-friendly), runs YOLOv8 on the laptop GPU, and:
  • shows a live window with bounding boxes + labels (what the robot sees),
  • publishes an annotated CompressedImage so the web UI / RViz can show it too,
  • publishes /detections (std_msgs/String, JSON) — a light, dependency-free
    list of {label, conf, x, y, w, h} per frame for any downstream consumer.

Why on the laptop: the Jetson Nano is too weak for real-time YOLO and the extra
load caused motor jitter. The laptop has a real GPU and the camera is already
on the ROS graph, so detection here costs the robot nothing.

Run:  bash run_detector.sh        (sets ROS env + launches this)
Setup once: bash setup_detector.sh
"""

import json
import os
import sys

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

try:
    import cv2
except ImportError:
    print("ERROR: OpenCV (cv2) not found. Run setup_detector.sh first.")
    sys.exit(1)

try:
    from ultralytics import YOLO
except ImportError:
    print("ERROR: ultralytics not found. Run: pip install ultralytics")
    sys.exit(1)


# Input: the Nano's color camera, compressed transport (small over WiFi).
IN_TOPIC   = '/camera/color/image_raw/compressed'
OUT_TOPIC  = '/detection/image/compressed'     # annotated, for web UI / RViz
DET_TOPIC  = '/detections'                       # JSON list, dependency-free

# Model — runs on the LAPTOP GPU, so we default to an ACCURATE model, not the
# tiny/fast one. YOLO11 is ultralytics' current-gen, more accurate than YOLOv8
# at the same size. Pick by your laptop GPU (all auto-download on first run):
#   yolo11n.pt  fastest,  least accurate   (weak GPU / CPU fallback)
#   yolo11s.pt  fast,     decent
#   yolo11m.pt  balanced, accurate         ← default (good on most laptop GPUs)
#   yolo11l.pt  slower,   very accurate
#   yolo11x.pt  slowest,  MOST accurate    (strong GPU; best detections)
# Override without editing: export HEXAPOD_YOLO_MODEL=yolo11x.pt
MODEL      = os.environ.get('HEXAPOD_YOLO_MODEL', 'yolo11m.pt')
CONF_MIN   = float(os.environ.get('HEXAPOD_YOLO_CONF', '0.40'))  # detection threshold
SHOW_WINDOW = True           # live cv2 window on the laptop


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.get_logger().info(f'Loading {MODEL} (first run auto-downloads it)…')
        self.model = YOLO(MODEL)             # GPU auto-used if torch sees CUDA
        self.names = self.model.names

        self.sub = self.create_subscription(
            CompressedImage, IN_TOPIC, self._on_image, qos_profile_sensor_data)
        self.img_pub = self.create_publisher(CompressedImage, OUT_TOPIC, 5)
        self.det_pub = self.create_publisher(String, DET_TOPIC, 5)

        self._n = 0
        if SHOW_WINDOW:
            # Resizable, decently-sized window (default auto-size can show as a
            # tiny black box under some Qt backends).
            cv2.namedWindow('Hexapod — YOLO detections', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Hexapod — YOLO detections', 960, 720)
        self.get_logger().info(
            f'Detector ready. In: {IN_TOPIC}  →  boxes window + {OUT_TOPIC}')

    def _on_image(self, msg: CompressedImage):
        # Decode JPEG → BGR
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            return

        # Inference (verbose off; GPU if available)
        results = self.model(frame, conf=CONF_MIN, verbose=False)[0]

        dets = []
        for b in results.boxes:
            x1, y1, x2, y2 = (int(v) for v in b.xyxy[0])
            conf = float(b.conf[0])
            cls = int(b.cls[0])
            label = self.names.get(cls, str(cls))
            # People highlighted green, everything else cyan
            color = (0, 255, 0) if label == 'person' else (255, 200, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f'{label} {conf:.2f}', (x1, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            dets.append({'label': label, 'conf': round(conf, 2),
                         'x': x1, 'y': y1, 'w': x2 - x1, 'h': y2 - y1})

        # Publish the detection list (JSON string — no vision_msgs dependency)
        self.det_pub.publish(String(data=json.dumps(dets)))

        # Publish the annotated frame (compressed) for the web UI / RViz
        ok, enc = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = enc.tobytes()
            self.img_pub.publish(out)

        # Live window on the laptop
        if SHOW_WINDOW:
            cv2.imshow('Hexapod — YOLO detections', frame)
            cv2.waitKey(1)

        self._n += 1
        if self._n % 30 == 0:
            ppl = sum(1 for d in dets if d['label'] == 'person')
            bright = float(frame.mean())   # ~0 = black frame, >40 = real image
            self.get_logger().info(
                f'frame {self._n}: {len(dets)} objects ({ppl} person) '
                f'| avg brightness {bright:.0f}/255 ({frame.shape[1]}x{frame.shape[0]})')


def main():
    rclpy.init()
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
