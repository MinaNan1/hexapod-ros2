"""
hexapod_teleop.py
Tkinter GUI with:
  - Roll / Pitch / Yaw / TX / TY / TZ sliders  → /hexapod/posture (Twist)
  - Walk / Stop / Forward / Backward / Rotate buttons → /hexapod/cmd (String)

Topic split (matches the web UI): pose lives on /hexapod/posture, motion lives
on /hexapod/body_pose. This GUI only poses the body via the sliders and drives
motion via the buttons, so it publishes posture + cmd.
"""

import math
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ── Colours ───────────────────────────────────────────────────────────────────
DARK   = '#1e1e2e'
PANEL  = '#2a2a3e'
ACCENT = '#7c6af7'
GREEN  = '#a6e3a1'
RED    = '#f38ba8'
YELLOW = '#f9e2af'
TEXT   = '#cdd6f4'
MUTED  = '#6c7086'
TROUGH = '#45475a'

SLIDERS = [
    # (label,           key,    lo,  hi,  unit,  res)
    ('Roll   (rx)',  'roll',  -30,  30,  '°',  0.5),
    ('Pitch  (ry)',  'pitch', -30,  30,  '°',  0.5),
    ('Yaw / Steer', 'yaw',   -45,  45,  '°',  0.5),
    ('TX',           'tx',   -50,  50,  'mm', 1.0),
    ('TY',           'ty',   -50,  50,  'mm', 1.0),
    ('TZ  (height)', 'tz',   -30,  30,  'mm', 0.5),
]


# ── ROS node ──────────────────────────────────────────────────────────────────

class TeleopNode(Node):
    def __init__(self):
        super().__init__('hexapod_teleop')
        self.pose_pub = self.create_publisher(Twist,  '/hexapod/posture', 10)
        self.cmd_pub  = self.create_publisher(String, '/hexapod/cmd',       10)

    def send_pose(self, roll, pitch, yaw, tx, ty, tz):
        msg = Twist()
        msg.angular.x = math.radians(roll)
        msg.angular.y = math.radians(pitch)
        msg.angular.z = math.radians(yaw)
        msg.linear.x  = float(tx)
        msg.linear.y  = float(ty)
        msg.linear.z  = float(tz)
        self.pose_pub.publish(msg)

    def send_cmd(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)


# ── GUI ───────────────────────────────────────────────────────────────────────

def build_gui(node: TeleopNode):
    root = tk.Tk()
    root.title('Hexapod Controller')
    root.configure(bg=DARK)
    root.geometry('560x640')
    root.resizable(False, False)

    # ── Header ────────────────────────────────────────────────────────────────
    tk.Label(root, text='🦀  Hexapod Controller',
             bg=DARK, fg=TEXT, font=('Helvetica', 16, 'bold')).pack(pady=(16, 2))
    tk.Label(root, text='Body Pose · Steering · Motion',
             bg=DARK, fg=MUTED, font=('Helvetica', 10)).pack(pady=(0, 10))

    # ── Motion control buttons ─────────────────────────────────────────────
    btn_row = tk.Frame(root, bg=DARK)
    btn_row.pack(pady=(0, 10))

    status_var = tk.StringVar(value='● WALKING')
    status_lbl = tk.Label(btn_row, textvariable=status_var,
                          bg=DARK, fg=GREEN, font=('Courier', 11, 'bold'))
    status_lbl.grid(row=0, column=0, columnspan=5, pady=(0, 8))

    def btn(parent, text, color, cmd_str, col, row=1):
        def cb():
            node.send_cmd(cmd_str)
            if cmd_str == 'stop':
                status_var.set('■ STOPPED')
                status_lbl.config(fg=RED)
            elif cmd_str == 'backward':
                status_var.set('◀ BACKWARD')
                status_lbl.config(fg=YELLOW)
            elif cmd_str == 'rotate_left':
                status_var.set('↺ ROTATE L')
                status_lbl.config(fg=ACCENT)
            elif cmd_str == 'rotate_right':
                status_var.set('↻ ROTATE R')
                status_lbl.config(fg=ACCENT)
            else:
                status_var.set('● WALKING')
                status_lbl.config(fg=GREEN)
        tk.Button(parent, text=text, command=cb,
                  bg=color, fg='white', font=('Helvetica', 10, 'bold'),
                  relief='flat', padx=10, pady=6, cursor='hand2',
                  width=9).grid(row=row, column=col, padx=3)

    btn(btn_row, '↺ Left',   ACCENT,    'rotate_left',  0)
    btn(btn_row, '◀ Back',   '#89b4fa', 'backward',     1)
    btn(btn_row, '■ Stop',   RED,       'stop',         2)
    btn(btn_row, '▶ Walk',   GREEN,     'walk',         3)
    btn(btn_row, '↻ Right',  ACCENT,    'rotate_right', 4)

    # ── Slider panel ──────────────────────────────────────────────────────────
    panel = tk.Frame(root, bg=PANEL, bd=0)
    panel.pack(fill='both', expand=True, padx=22, pady=4)

    vars_: dict[str, tk.DoubleVar] = {}

    def on_change(*_):
        node.send_pose(
            vars_['roll'].get(), vars_['pitch'].get(), vars_['yaw'].get(),
            vars_['tx'].get(),   vars_['ty'].get(),    vars_['tz'].get())

    for row, (label, key, lo, hi, unit, res) in enumerate(SLIDERS):
        if row == 3:   # separator between rotation and translation
            tk.Frame(panel, bg=TROUGH, height=1).grid(
                row=row * 2 - 1, column=0, columnspan=4,
                sticky='ew', padx=10, pady=4)

        tk.Label(panel, text=label, bg=PANEL, fg=TEXT,
                 font=('Helvetica', 11), width=15, anchor='w').grid(
                 row=row * 2, column=0, padx=(14, 4), pady=(10, 2), sticky='w')

        v = tk.DoubleVar(value=0.0)
        vars_[key] = v

        tk.Scale(panel, variable=v, from_=lo, to=hi,
                 orient='horizontal', resolution=res, length=220,
                 bg=PANEL, fg=TEXT, troughcolor=TROUGH,
                 activebackground=ACCENT, highlightbackground=PANEL,
                 bd=0, command=on_change).grid(
                 row=row * 2, column=1, padx=4, pady=(10, 2))

        tk.Label(panel, textvariable=v, bg=PANEL, fg=ACCENT,
                 font=('Courier', 10, 'bold'), width=6, anchor='e').grid(
                 row=row * 2, column=2, padx=(0, 2))

        tk.Label(panel, text=unit, bg=PANEL, fg=MUTED,
                 font=('Helvetica', 10), width=3, anchor='w').grid(
                 row=row * 2, column=3, padx=(0, 10))

    # ── Reset ─────────────────────────────────────────────────────────────────
    def reset():
        for v in vars_.values():
            v.set(0.0)
        on_change()

    tk.Button(root, text='  Reset Pose  ', command=reset,
              bg=ACCENT, fg='white', font=('Helvetica', 11, 'bold'),
              relief='flat', padx=16, pady=7, cursor='hand2',
              activebackground='#6a58e0').pack(pady=14)

    # ── Note about Yaw slider ────────────────────────────────────────────────
    tk.Label(root,
             text='ℹ  Yaw slider tilts the body · use ↺/↻ to turn while walking',
             bg=DARK, fg=MUTED, font=('Helvetica', 9)).pack(pady=(0, 10))

    def on_close():
        rclpy.shutdown()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    root.mainloop()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()
    build_gui(node)


if __name__ == '__main__':
    main()