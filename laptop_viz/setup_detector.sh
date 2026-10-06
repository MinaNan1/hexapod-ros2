#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# setup_detector.sh — ONE-TIME setup on the LAPTOP for Phase 4 YOLO detection.
# Installs ultralytics (YOLOv8) into the laptop's Python. On an x86 laptop with
# an NVIDIA GPU this is a clean pip install and YOLO will use the GPU
# automatically. The model file (yolov8n.pt, ~6 MB) auto-downloads on first run.
# ─────────────────────────────────────────────────────────────────────────────
set -e
echo "==> Installing ultralytics (YOLOv8) + deps on the laptop…"
pip install --user ultralytics opencv-python

echo ""
echo "==> Checking GPU visibility (optional but recommended)…"
python3 - <<'PY'
try:
    import torch
    print(f"   torch {torch.__version__}  CUDA available: {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"   GPU: {torch.cuda.get_device_name(0)}")
    else:
        print("   No CUDA → YOLO will run on CPU (slower but works).")
except Exception as e:
    print(f"   torch check skipped: {e}")
PY

echo ""
echo "Done. Launch detection with:  bash run_detector.sh"
echo "(the robot stack must be running and publishing /camera/color/image_raw/compressed)"
