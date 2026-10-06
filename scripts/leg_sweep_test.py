#!/usr/bin/env python3
"""
leg_sweep_test.py — RAW servo strength test. Bypasses ROS, Docker, the gait,
and the rate limiter entirely: writes Hiwonder HID packets straight to
/dev/hiwonder, sweeping the FRONT TWO legs' femur (lift) + tibia servos between
their extremes, looping.

Purpose: isolate hardware (servos + power + board) from software. If the legs
sweep STRONGLY and fully here, the servos/power are fine and the weak walking is
a software problem (command rate / rate limiter / CPU starvation). If they're
still weak/saggy here, it's power/electrical (battery, connector, supply).

IMPORTANT — stop the ROS stack first so it isn't also writing to the bus:
    bash ~/hexapod-ros2/scripts/stop_all.sh
    python3 ~/hexapod-ros2/scripts/leg_sweep_test.py
    (Ctrl-C to stop)
"""

import time

HID_DEVICE = '/dev/hiwonder'      # falls back to scanning if missing

# Front legs: r1 (front-right) femur=2 tibia=3 ; l3 (front-left) femur=17 tibia=18
FEMUR_IDS = [2, 17]
TIBIA_IDS = [3, 18]

# Raw servo range is 0..1000. Sweep femurs across a WIDE band so a healthy servo
# makes a big, obvious move. Tibias follow so the whole leg clearly raises/lowers.
FEMUR_HIGH, FEMUR_LOW = 750, 250
TIBIA_HIGH, TIBIA_LOW = 700, 300
MOVE_MS = 600                      # time the servo is told to take per move
PAUSE_S = 1.0                     # wait between moves (let it fully arrive)


def build_move_packet(servo_positions, move_time_ms):
    num = len(servo_positions)
    params = [num, move_time_ms & 0xFF, (move_time_ms >> 8) & 0xFF]
    for sid, pos in servo_positions.items():
        params += [sid, pos & 0xFF, (pos >> 8) & 0xFF]
    length = num * 3 + 5
    payload = [0x55, 0x55, length, 0x03] + params
    report = [0x00] + payload + [0x00] * (64 - len(payload))
    return bytes(report[:65])


def find_device():
    import os
    if os.path.exists(HID_DEVICE):
        return HID_DEVICE
    for d in sorted(os.listdir('/dev')):
        if d.startswith('hidraw'):
            return f'/dev/{d}'
    raise SystemExit('No /dev/hiwonder or /dev/hidraw* found — is the board plugged in?')


def main():
    dev = find_device()
    print(f'Opening {dev} … (Ctrl-C to stop)')
    hid = open(dev, 'wb')

    def send(positions, ms):
        hid.write(build_move_packet(positions, ms))
        hid.flush()

    up = {}
    for i in FEMUR_IDS: up[i] = FEMUR_HIGH
    for i in TIBIA_IDS: up[i] = TIBIA_HIGH
    down = {}
    for i in FEMUR_IDS: down[i] = FEMUR_LOW
    for i in TIBIA_IDS: down[i] = TIBIA_LOW

    n = 0
    while True:
        print(f'[{n}] RAISE front legs  femur→{FEMUR_HIGH} tibia→{TIBIA_HIGH}')
        send(up, MOVE_MS)
        time.sleep(PAUSE_S)
        print(f'[{n}] LOWER front legs  femur→{FEMUR_LOW} tibia→{TIBIA_LOW}')
        send(down, MOVE_MS)
        time.sleep(PAUSE_S)
        n += 1


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nstopped.')
