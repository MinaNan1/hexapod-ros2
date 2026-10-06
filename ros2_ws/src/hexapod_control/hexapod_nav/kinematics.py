"""
Hexapod Inverse Kinematics — dimensions from URDF (crab_model.xacro)
"""
import numpy as np
import math

class HexapodKinematics:

    def __init__(self):
        # Leg segment lengths from URDF (metres)
        self.l1 = 0.0698   # coxa
        self.l2 = 0.0759   # femur
        self.l3 = 0.1415   # tibia

        # Hip positions [body_x, body_y, coxa_angle] from URDF
        self.LEG_CONFIGS = {
            'fr': ( 0.096,  -0.10419,  math.atan2(-0.10419,  0.096)),
            'mr': ( 0.0,    -0.11025, -math.pi/2),
            'rr': (-0.096,  -0.10419,  math.atan2(-0.10419, -0.096)),
            'fl': ( 0.096,   0.10419,  math.atan2( 0.10419,  0.096)),
            'ml': ( 0.0,     0.11025,  math.pi/2),
            'rl': (-0.096,   0.10419,  math.atan2( 0.10419, -0.096)),
        }

        self.TRIPOD_A = ['fr', 'rr', 'ml']
        self.TRIPOD_B = ['mr', 'fl', 'rl']

        # Gait parameters
        self.STEP_LENGTH = 0.04
        self.STEP_HEIGHT = 0.03

        # Standing joint angles (verified: puts feet on ground at body height 0.092m)
        self.STANCE_THETA1 = 0.0
        self.STANCE_THETA2 = -0.1
        self.STANCE_THETA3 = -0.55

    def solve_ik(self, leg_name, foot_x, foot_y, foot_z):
        bx, by, coxa_angle = self.LEG_CONFIGS[leg_name]
        dx = foot_x - bx
        dy = foot_y - by
        dz = foot_z

        theta1 = math.atan2(dy, dx) - coxa_angle
        r = math.sqrt(dx**2 + dy**2) - self.l1
        if r < 0.001:
            r = 0.001

        D = (r**2 + dz**2 - self.l2**2 - self.l3**2) / (2 * self.l2 * self.l3)
        if abs(D) > 1.0:
            return None

        theta3 = math.atan2(-math.sqrt(max(0, 1 - D**2)), D)
        theta2 = math.atan2(dz, r) - math.atan2(
            self.l3 * math.sin(theta3),
            self.l2 + self.l3 * math.cos(theta3))

        return (theta1, theta2, theta3)

    def stance_foot_pos(self, leg_name):
        """Compute foot position from stance joint angles via forward kinematics."""
        bx, by, angle = self.LEG_CONFIGS[leg_name]
        t2, t3 = self.STANCE_THETA2, self.STANCE_THETA3
        r = self.l2 * math.cos(t2) + self.l3 * math.cos(t2 + t3)
        z = self.l2 * math.sin(t2) + self.l3 * math.sin(t2 + t3)
        total_reach = self.l1 + r
        fx = bx + total_reach * math.cos(angle)
        fy = by + total_reach * math.sin(angle)
        return (fx, fy, z)

    def default_stance(self):
        return {leg: self.stance_foot_pos(leg) for leg in self.LEG_CONFIGS}
