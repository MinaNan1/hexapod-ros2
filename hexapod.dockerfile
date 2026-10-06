FROM ros:humble

# Avoid interactive prompts during apt installs
ENV DEBIAN_FRONTEND=noninteractive

# ── System + ROS packages ────────────────────────────────────────────────
RUN apt-get update && apt-get install -y software-properties-common \
    && add-apt-repository universe \
    && apt-get update

RUN apt-get install -y \
    # ROS core
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-rosbridge-suite \
    ros-humble-slam-toolbox \
    ros-humble-joy \
    ros-humble-hls-lfcd-lds-driver \
    ros-humble-topic-tools \
    # Build tools
    python3-colcon-common-extensions \
    python3-pip \
    python3-tk \
    # Utilities
    nano \
    curl \
    git \
    && rm -rf /var/lib/apt/lists/*

# ── Python packages (path planning library) ─────────────────────────────
RUN pip3 install numpy scipy matplotlib Pillow flask

# ── I2C support for the MPU-9250 IMU (separate layer → fast rebuilds) ────
RUN apt-get update && apt-get install -y \
    i2c-tools \
    python3-smbus \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install smbus2

# ── Astra Pro depth camera — OrbbecSDK_ROS2 build deps + image transport ─
# Phase 1 of the perception integration. The actual ROS2 package
# (`orbbec_camera`) is cloned into `ros2_ws/src/` by `scripts/setup_orbbec.sh`
# and built with the rest of the workspace by `build_ws.sh`. The deps below
# match the OrbbecSDK_ROS2 **main** branch (OpenNI v1.x) install instructions
# — the original Astra Pro is a legacy OpenNI device and the v2-main branch
# dropped support for OpenNI. Plus the image_transport plugins we use to
# keep WiFi load down on the laptop side.
RUN apt-get update && apt-get install -y \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    libudev-dev \
    libgflags-dev \
    nlohmann-json3-dev \
    libgoogle-glog-dev \
    libeigen3-dev \
    libssl-dev \
    libdw-dev \
    cmake \
    build-essential \
    # OrbbecSDK_ROS2 main branch extra deps (per its README)
    ros-humble-camera-info-manager \
    ros-humble-diagnostic-updater \
    ros-humble-diagnostic-msgs \
    ros-humble-statistics-msgs \
    ros-humble-backward-ros \
    # ROS image_transport stack — compressed + compressedDepth keep the
    # laptop link from saturating; depth_image_proc is needed by RTAB-Map
    # in Phase 2 so we install it here once.
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport \
    ros-humble-compressed-depth-image-transport \
    ros-humble-image-publisher \
    ros-humble-image-proc \
    ros-humble-depth-image-proc \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    # UVC color driver — the original Astra Pro routes color through a
    # standard UVC interface (/dev/video0), which the OrbbecSDK_ROS2 main
    # branch can't drive directly. v4l2_camera handles it, runs in parallel
    # with orbbec_camera (depth-only).
    ros-humble-v4l2-camera \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# ── Phase 2: 3D SLAM stack (RTAB-Map) ────────────────────────────────────
# rtabmap_ros is the modern apt-installable RTAB-Map for ROS 2. We use the
# binary instead of the vendored source tree in ros2_ws/src/rtabmap_ros/
# because the binary is faster to install and just as capable for our
# robot-scale mapping. build_ws.sh already excludes the vendored sources.
RUN apt-get update && apt-get install -y \
    ros-humble-rtabmap-ros \
    ros-humble-rtabmap \
    ros-humble-octomap-msgs \
    && rm -rf /var/lib/apt/lists/*

# ── Convenience: auto-source ROS on every shell ─────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# ── Entry point stays as bash so we can run multiple commands ────────────
CMD ["/bin/bash"]
