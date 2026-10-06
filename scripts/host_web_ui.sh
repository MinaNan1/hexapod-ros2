#!/bin/bash
# HOST terminal — Flask web server (run on Jetson, NOT inside container)
cd ~/hexapod-ros2/ros2_ws/src/hexapod_control/web_ui
python3 app.py
# Then open browser at http://localhost:5000
