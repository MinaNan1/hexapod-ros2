#!/usr/bin/env python3
"""
servo_feedback_test.py — validate HX-35H position FEEDBACK over the Hiwonder
USB-HID controller, and measure how fast a full 18-servo read is.

READ-ONLY: it does not move anything. Run it with the servo bridge STOPPED so it
has exclusive access to /dev/hiwonder (servos stay powered and hold position):

    # on the Jetson:
    docker exec -it hexapod_ros bash
    pkill -f hiwonder_servo_bridge        # free the device (keep servos powered)
    python3 /hexapodd/scripts/servo_feedback_test.py

Protocol (best-guess, mirrors the bridge's write path):
  request : 0x55 0x55 [count+3] 0x15(CMD_MULT_SERVO_POS_READ) [count] [id1..idN]
  reply   : 0x55 0x55 [len] 0x15 [count] [id1 lo hi] [id2 lo hi] ...
  position = lo + (hi<<8)   (0..1000 ≈ 0..240°)

If the parse looks wrong, the RAW bytes are printed so we can fix the format.
"""
import os, glob, select, time, sys

IDS = list(range(1, 19))   # 18 servos (LEGS r1..l3 = ids 1..18)
HIWONDER_VID, HIWONDER_PID = "0483", "5750"

def find_device():
    if os.path.exists('/dev/hiwonder'):
        return '/dev/hiwonder'
    for node in sorted(glob.glob('/dev/hidraw*')):
        try:
            with open(f'/sys/class/hidraw/{os.path.basename(node)}/device/uevent') as f:
                txt = f.read()
            if f":{HIWONDER_VID.lower()}".lower() in txt.lower() and HIWONDER_PID.lower() in txt.lower():
                return node
        except OSError:
            pass
    return None

def build_read_packet(ids):
    count = len(ids)
    length = count + 3
    payload = [0x55, 0x55, length, 0x15, count] + list(ids)
    report = [0x00] + payload + [0x00] * (64 - len(payload))
    return bytes(report[:65])

def parse_reply(data):
    """Find 0x55 0x55 header and pull out {id: position}."""
    b = list(data)
    for i in range(len(b) - 4):
        if b[i] == 0x55 and b[i+1] == 0x55 and b[i+3] == 0x15:
            count = b[i+4]
            out = {}
            p = i + 5
            for _ in range(count):
                if p + 2 >= len(b): break
                sid, lo, hi = b[p], b[p+1], b[p+2]
                out[sid] = lo + (hi << 8)
                p += 3
            return out
    return {}

def read_once(fd, ids, timeout=0.5):
    # drain any stale input
    while select.select([fd], [], [], 0)[0]:
        try: os.read(fd, 64)
        except OSError: break
    t0 = time.time()
    os.write(fd, build_read_packet(ids))
    r, _, _ = select.select([fd], [], [], timeout)
    if not r:
        return None, time.time() - t0, b''
    data = os.read(fd, 64)
    return parse_reply(data), time.time() - t0, data

def main():
    path = find_device()
    if not path:
        print("❌ Hiwonder device not found (/dev/hiwonder or VID:PID 0483:5750).")
        print("   Is the board plugged in? Is the bridge still holding it? (pkill it)")
        sys.exit(1)
    print(f"🔌 device: {path}")
    try:
        fd = os.open(path, os.O_RDWR)
    except PermissionError:
        print(f"❌ permission denied — run: sudo chmod 666 {path}"); sys.exit(1)

    print("\n--- single read of all 18 servos ---")
    pos, dt, raw = read_once(fd, IDS)
    print(f"raw reply ({len(raw)} bytes): {raw.hex(' ')}")
    if not pos:
        print("⚠ no/!parseable reply. The 0x15 read may need a different format, OR the")
        print("  board returns servos one-at-a-time. Trying single-servo reads next…")
        for sid in IDS[:3]:
            p, d, rw = read_once(fd, [sid])
            print(f"  id {sid}: {p}  raw={rw.hex(' ')}  ({d*1000:.0f} ms)")
    else:
        print(f"parsed {len(pos)}/18 servos in {dt*1000:.0f} ms:")
        for sid in IDS:
            print(f"  servo {sid:2d}: {pos.get(sid,'—')}")

    print("\n--- read-rate over 20 reads (this caps a feedback-paced gait) ---")
    ts = []
    for _ in range(20):
        _, dt, _ = read_once(fd, IDS); ts.append(dt)
    ts.sort()
    print(f"  per full-18 read: min {min(ts)*1000:.0f}  median {ts[len(ts)//2]*1000:.0f}  "
          f"max {max(ts)*1000:.0f} ms  → ~{1/ (ts[len(ts)//2]+1e-6):.0f} reads/s")
    os.close(fd)

if __name__ == "__main__":
    main()
