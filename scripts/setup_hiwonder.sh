#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# setup_hiwonder.sh — one-time (per-Jetson) Hiwonder bus-servo controller setup.
#
# Run on the JETSON HOST (NOT inside the docker container, NOT over the bridge):
#     sudo bash ~/hexapod-ros2/scripts/setup_hiwonder.sh
#
# What it does:
#   1. Installs a udev rule so the Hiwonder controller (USB 0483:5750) always
#      appears as a stable /dev/hiwonder symlink with 0666 perms — no more
#      hunting for the right /dev/hidrawN and no more `chmod` every boot.
#   2. If the board's HID interface is enumerated but has no driver bound
#      (a state we hit on the Nano), it rebinds usbhid so the hidraw node
#      appears. If that fails, it tells you to replug the USB cable.
#   3. Verifies /dev/hiwonder exists at the end.
# ─────────────────────────────────────────────────────────────────────────────
set -u

VID=0483
PID=5750
RULE=/etc/udev/rules.d/99-hiwonder.rules

if [ "$(id -u)" -ne 0 ]; then
    echo "Please run as root:  sudo bash $0"
    exit 1
fi

echo "==> 1. Installing udev rule ($RULE)"
cat > "$RULE" <<EOF
# Hiwonder Serial Bus Servo Controller (MindMotion SOC). Stable name + perms.
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="$VID", ATTRS{idProduct}=="$PID", MODE="0666", SYMLINK+="hiwonder"
EOF
udevadm control --reload-rules
udevadm trigger
sleep 1

# Find the board's USB interface (e.g. 1-2.1:1.0) and whether it has a driver.
echo "==> 2. Checking the board's USB / HID binding"
USB_DEV=""
for d in /sys/bus/usb/devices/*/idProduct; do
    dir=$(dirname "$d")
    if [ "$(cat "$d" 2>/dev/null)" = "$PID" ] && [ "$(cat "$dir/idVendor" 2>/dev/null)" = "$VID" ]; then
        USB_DEV="$dir"
        break
    fi
done

if [ -z "$USB_DEV" ]; then
    echo "   ✗ Board (USB $VID:$PID) is NOT on the USB bus at all."
    echo "     Check the USB cable and the controller's own power supply, then re-run."
    exit 1
fi
echo "   ✓ Board present at $(basename "$USB_DEV")"

have_hidraw() {  # is there a hidraw node for 0483:5750 right now?
    for h in /sys/class/hidraw/hidraw*; do
        grep -qi "0000$VID:0000$PID" "$h"/device/uevent 2>/dev/null && return 0
    done
    return 1
}

IFACE="$(basename "$USB_DEV"):1.0"
DRIVER=$(readlink "$USB_DEV:1.0/driver" 2>/dev/null | sed 's#.*/##')
if [ -n "$DRIVER" ] && have_hidraw; then
    echo "   ✓ HID interface bound to '$DRIVER'"
else
    echo "   ! HID interface $IFACE not bound to a hidraw — recovering…"

    # (a) cheapest: just bind usbhid to the interface.
    echo -n "$IFACE" > /sys/bus/usb/drivers/usbhid/bind 2>/dev/null && echo "     · usbhid bind issued"
    sleep 1; udevadm trigger; sleep 1

    # (b) if still nothing, force a full software re-enumeration of the USB
    #     device (deauthorize → authorize). This re-runs enumeration + driver
    #     binding without anyone touching the cable — usually clears the
    #     Nano's "interface present but unbound" state.
    if ! have_hidraw && [ -w "$USB_DEV/authorized" ]; then
        echo "     · re-enumerating $(basename "$USB_DEV") via sysfs authorized toggle…"
        echo 0 > "$USB_DEV/authorized"; sleep 1
        echo 1 > "$USB_DEV/authorized"; sleep 2
        udevadm trigger; sleep 1
    fi

    if have_hidraw; then
        echo "   ✓ hidraw node now present"
    else
        echo "   ✗ still no hidraw node — UNPLUG and REPLUG the Hiwonder USB cable, then re-run."
    fi
fi

udevadm trigger
sleep 1

echo "==> 3. Result"
if [ -e /dev/hiwonder ]; then
    echo "   ✓ /dev/hiwonder -> $(readlink -f /dev/hiwonder)  $(ls -l /dev/hiwonder | awk '{print $1}')"
    echo ""
    echo "Done. The bridge will now find the board automatically as /dev/hiwonder."
else
    echo "   ✗ /dev/hiwonder did not appear."
    echo "     The board enumerates but its hidraw node isn't coming up."
    echo "     Try: unplug + replug the Hiwonder USB cable, then re-run this script."
    echo "     Current hidraw nodes:"
    for h in /dev/hidraw*; do
        n=$(basename "$h"); u=/sys/class/hidraw/$n/device/uevent
        echo "       $h  $(grep -m1 HID_NAME= "$u" 2>/dev/null | cut -d= -f2)"
    done
    exit 1
fi
