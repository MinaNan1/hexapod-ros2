#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────
# setup_orbbec.sh — one-time Astra Pro depth camera setup.
#
# IMPORTANT: clones the OrbbecSDK_ROS2 **main** branch (v1.x, OpenNI-based),
# not the default v2-main. The v2 branch dropped support for OpenNI devices
# and the original Astra Pro (USB 2bc5:0403 + 2bc5:0501) is an OpenNI device.
#
# Does three things on the JETSON HOST:
#   1. Clones orbbec/OrbbecSDK_ROS2 main branch into ros2_ws/src/
#   2. Installs Orbbec udev rules so /dev access works in the container
#   3. Verifies everything (loud errors, not silent set -e exits)
#
# Run this ONCE on the Jetson host (not inside the container):
#   bash ~/hexapod-ros2/scripts/setup_orbbec.sh
#
# Then rebuild the workspace inside the container:
#   docker exec -it hexapod_ros bash /hexapodd/build_ws.sh
# ─────────────────────────────────────────────────────────────────────────

# NOTE: deliberately NOT using `set -e` — we want to print errors and keep
# going so the user sees the full picture if something fails.

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SRC_DIR="$PROJECT_DIR/ros2_ws/src"
ORBBEC_REPO="https://github.com/orbbec/OrbbecSDK_ROS2.git"
ORBBEC_DIR="$SRC_DIR/OrbbecSDK_ROS2"
ORBBEC_BRANCH="main"

echo "──────────────────────────────────────────────────────────────"
echo "  Astra Pro / OrbbecSDK_ROS2 (main branch, OpenNI v1.x) setup"
echo "──────────────────────────────────────────────────────────────"
echo "  Project:        $PROJECT_DIR"
echo "  Clone target:   $ORBBEC_DIR"
echo "  Repo + branch:  $ORBBEC_REPO  ($ORBBEC_BRANCH)"
echo ""

# ── Sanity check 1: git available on the host ────────────────────────────
if ! command -v git >/dev/null 2>&1; then
    echo "ERROR: 'git' is not installed on the Jetson host."
    echo "Fix:   sudo apt update && sudo apt install -y git"
    echo "Then re-run this script."
    exit 1
fi
echo "[ok] git: $(git --version)"

# ── Sanity check 2: ros2_ws/src exists ──────────────────────────────────
if [ ! -d "$SRC_DIR" ]; then
    echo "ERROR: $SRC_DIR does not exist."
    echo "       Are you running this from the right project tree?"
    exit 1
fi
echo "[ok] workspace src dir: $SRC_DIR"

# ── 1/3: Clone (or update) OrbbecSDK_ROS2 main branch ──────────────────
if [ -d "$ORBBEC_DIR/.git" ]; then
    echo ""
    echo "[1/3] OrbbecSDK_ROS2 already cloned at $ORBBEC_DIR"
    echo "      Checking branch + pulling latest..."
    (cd "$ORBBEC_DIR" && git checkout "$ORBBEC_BRANCH" && git pull --ff-only)
    if [ $? -ne 0 ]; then
        echo "WARN: git pull failed. Continuing with whatever's on disk."
    fi
else
    echo ""
    echo "[1/3] Cloning OrbbecSDK_ROS2 ($ORBBEC_BRANCH branch) into $ORBBEC_DIR ..."
    git clone --branch "$ORBBEC_BRANCH" --depth 1 "$ORBBEC_REPO" "$ORBBEC_DIR"
    if [ $? -ne 0 ]; then
        echo "ERROR: git clone failed. Possible causes:"
        echo "  - No network on the Jetson"
        echo "  - GitHub TLS certs (old git: 'sudo apt install ca-certificates')"
        echo "  - DNS issue ('ping github.com')"
        exit 1
    fi
fi

# Verify the clone actually produced what we need
if [ ! -f "$ORBBEC_DIR/orbbec_camera/package.xml" ]; then
    echo "ERROR: $ORBBEC_DIR/orbbec_camera/package.xml is missing after clone."
    echo "       The repo layout may have changed. Contents of $ORBBEC_DIR:"
    ls -la "$ORBBEC_DIR" 2>/dev/null
    exit 1
fi
echo "[ok] orbbec_camera package.xml present"

# ── 2/3: Install udev rules on the host ─────────────────────────────────
UDEV_SCRIPT="$ORBBEC_DIR/orbbec_camera/scripts/install_udev_rules.sh"

if [ ! -f "$UDEV_SCRIPT" ]; then
    echo ""
    echo "WARN: $UDEV_SCRIPT not found."
    echo "      Looking for any udev rules in the repo..."
    find "$ORBBEC_DIR" -name "*.rules" -o -name "install_udev*" 2>/dev/null
    echo "WARN: continuing without udev rules — camera may need to be run as root."
else
    echo ""
    echo "[2/3] Installing Orbbec udev rules (sudo required)"
    (cd "$ORBBEC_DIR/orbbec_camera/scripts" && sudo bash install_udev_rules.sh)
    if [ $? -ne 0 ]; then
        echo "WARN: install_udev_rules.sh returned non-zero. Trying manual fallback..."
        # Manual fallback: find the .rules file and copy it ourselves
        for rules in "$ORBBEC_DIR"/orbbec_camera/scripts/*.rules; do
            if [ -f "$rules" ]; then
                echo "      Copying $rules → /etc/udev/rules.d/"
                sudo cp "$rules" /etc/udev/rules.d/
            fi
        done
    fi
    sudo udevadm control --reload-rules 2>/dev/null
    sudo udevadm trigger 2>/dev/null
    echo "[ok] udev rules reloaded"
fi

# ── 3/3: USB sanity check ───────────────────────────────────────────────
echo ""
echo "[3/3] Verifying the Astra Pro is currently enumerated..."
ORBBEC_USB=$(lsusb | grep -iE 'orbbec|2bc5' || true)
if [ -z "$ORBBEC_USB" ]; then
    echo "WARN: No Orbbec USB device detected right now."
    echo "      That's OK if it isn't plugged in — but make sure to plug it in"
    echo "      before running launch_all.sh."
else
    echo "$ORBBEC_USB" | sed 's/^/      /'
    if echo "$ORBBEC_USB" | grep -q '2bc5:0403'; then
        echo "      ↳ UVC color interface detected (2bc5:0403)"
    fi
    if echo "$ORBBEC_USB" | grep -q '2bc5:0501'; then
        echo "      ↳ OpenNI depth interface detected (2bc5:0501)"
    fi
fi

echo ""
echo "──────────────────────────────────────────────────────────────"
echo "  Done. Next steps:"
echo "    1. UNPLUG + REPLUG the Astra Pro (new udev rules apply)"
echo "    2. Rebuild the workspace:"
echo "         docker exec -it hexapod_ros bash /hexapodd/build_ws.sh"
echo "       (takes 10-15 min — OrbbecSDK_ROS2 builds the SDK from source)"
echo "    3. Restart the stack:"
echo "         bash $PROJECT_DIR/scripts/stop_all.sh"
echo "         bash $PROJECT_DIR/scripts/launch_all.sh"
echo "──────────────────────────────────────────────────────────────"
