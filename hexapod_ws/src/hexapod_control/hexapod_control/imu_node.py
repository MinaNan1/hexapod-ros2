"""
imu_node.py
===========
Driver for the MPU-9250 / MPU-6500 / MPU-9255 IMU over I2C.

Reads the accelerometer + gyroscope, fuses them with a complementary
filter to get a drift-free roll/pitch estimate, and publishes a
sensor_msgs/Imu on /imu/data.

We only fuse accel + gyro (no magnetometer): roll/pitch are all the
self-balancing controller needs, and they are observable from gravity
alone. Yaw is left at zero (it would need the magnetometer + hard/soft
iron calibration, which leveling does not require).

Wiring (MPU-9250 breakout → Jetson Nano 40-pin header):
    VCC  → pin 1  (3.3V)        ← use 3.3V, not 5V, to match Jetson I2C logic
    GND  → pin 6  (GND)
    SCL  → pin 5  (I2C1 SCL)
    SDA  → pin 3  (I2C1 SDA)
    NCS  → pin 1  (3.3V)        ← hold high to select I2C mode
    AD0  → GND (addr 0x68)  or  3.3V (addr 0x69)
    FSYNC, EDA, ECL, INT → leave unconnected

That puts the device on I2C bus 1 (/dev/i2c-1) at address 0x68.
Confirm with:  i2cdetect -y -r 1
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# ── MPU-9250 register map ──────────────────────────────────────────────────────
WHO_AM_I      = 0x75
PWR_MGMT_1    = 0x6B
SMPLRT_DIV    = 0x19
CONFIG        = 0x1A
GYRO_CONFIG   = 0x1B
ACCEL_CONFIG  = 0x1C
ACCEL_CONFIG2 = 0x1D
ACCEL_XOUT_H  = 0x3B

# Sensitivity for the full-scale ranges we configure below
ACCEL_SENS = 16384.0   # LSB/g    at ±2 g
GYRO_SENS  = 131.0     # LSB/dps  at ±250 °/s
G_MS2      = 9.80665   # m/s² per g


def _open_bus(bus_num):
    """Open an I2C bus, trying smbus2 then smbus. Returns (bus, lib_name) or (None, None)."""
    try:
        import smbus2
        return smbus2.SMBus(bus_num), 'smbus2'
    except Exception:
        pass
    try:
        import smbus
        return smbus.SMBus(bus_num), 'smbus'
    except Exception:
        pass
    return None, None


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('i2c_bus', 1)          # /dev/i2c-1 on the 40-pin header
        self.declare_parameter('i2c_addr', 0x68)      # 0x68 (AD0 low) or 0x69 (AD0 high)
        self.declare_parameter('rate_hz', 50.0)       # publish rate
        self.declare_parameter('alpha', 0.98)         # complementary filter weight (gyro)
        self.declare_parameter('calib_samples', 200)  # gyro-bias samples at startup
        self.declare_parameter('frame_id', 'imu_link')

        self.bus_num    = self.get_parameter('i2c_bus').value
        self.addr       = self.get_parameter('i2c_addr').value
        self.rate_hz    = self.get_parameter('rate_hz').value
        self.alpha      = self.get_parameter('alpha').value
        self.frame_id   = self.get_parameter('frame_id').value
        n_calib         = self.get_parameter('calib_samples').value

        self.pub = self.create_publisher(Imu, '/imu/data', 10)

        # ── Filter state ────────────────────────────────────────────────────
        self.roll  = 0.0
        self.pitch = 0.0
        self.gyro_bias = [0.0, 0.0, 0.0]
        self._last_t = None
        self._ok = False

        # ── Open the I2C bus ────────────────────────────────────────────────
        self.bus, lib = _open_bus(self.bus_num)
        if self.bus is None:
            self.get_logger().error(
                'No I2C library found. Install one in the container:\n'
                '  pip3 install smbus2   (or)   apt install python3-smbus\n'
                'IMU node will idle and publish nothing until then.')
            return

        try:
            self._init_mpu()
            self._calibrate_gyro(n_calib)
            self._ok = True
            self.get_logger().info(
                f'MPU-9250 ready on /dev/i2c-{self.bus_num} @ 0x{self.addr:02x} '
                f'(via {lib}) — publishing /imu/data at {self.rate_hz:.0f} Hz')
        except Exception as e:
            self.get_logger().error(
                f'Could not init MPU-9250 on /dev/i2c-{self.bus_num} @ '
                f'0x{self.addr:02x}: {e}\n'
                'Check wiring and run:  i2cdetect -y -r ' + str(self.bus_num))
            return

        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

    # ── Low-level I2C ───────────────────────────────────────────────────────

    def _init_mpu(self):
        # Verify identity (don't hard-fail on unknown — clones vary)
        who = self.bus.read_byte_data(self.addr, WHO_AM_I)
        known = {0x71: 'MPU-9250', 0x73: 'MPU-9255', 0x70: 'MPU-6500', 0x68: 'MPU-6050'}
        self.get_logger().info(
            f'WHO_AM_I = 0x{who:02x} ({known.get(who, "unknown clone — continuing")})')

        # Wake up, select best available clock
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x01)
        # Sample rate divider: 1 kHz / (1+4) = 200 Hz internal
        self.bus.write_byte_data(self.addr, SMPLRT_DIV, 0x04)
        # DLPF ~41 Hz for gyro/temp
        self.bus.write_byte_data(self.addr, CONFIG, 0x03)
        # Gyro full scale ±250 °/s
        self.bus.write_byte_data(self.addr, GYRO_CONFIG, 0x00)
        # Accel full scale ±2 g
        self.bus.write_byte_data(self.addr, ACCEL_CONFIG, 0x00)
        # Accel DLPF ~44 Hz
        self.bus.write_byte_data(self.addr, ACCEL_CONFIG2, 0x03)

    @staticmethod
    def _to_signed(high, low):
        val = (high << 8) | low
        return val - 65536 if val >= 0x8000 else val

    def _read_raw(self):
        """Return (ax, ay, az in g) and (gx, gy, gz in °/s)."""
        block = self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT_H, 14)
        ax = self._to_signed(block[0],  block[1])  / ACCEL_SENS
        ay = self._to_signed(block[2],  block[3])  / ACCEL_SENS
        az = self._to_signed(block[4],  block[5])  / ACCEL_SENS
        # block[6:8] = temperature (skipped)
        gx = self._to_signed(block[8],  block[9])  / GYRO_SENS
        gy = self._to_signed(block[10], block[11]) / GYRO_SENS
        gz = self._to_signed(block[12], block[13]) / GYRO_SENS
        return (ax, ay, az), (gx, gy, gz)

    def _calibrate_gyro(self, n):
        """Average n samples while the robot is still to find gyro bias."""
        if n <= 0:
            return
        sums = [0.0, 0.0, 0.0]
        for _ in range(n):
            _, g = self._read_raw()
            sums[0] += g[0]; sums[1] += g[1]; sums[2] += g[2]
        self.gyro_bias = [s / n for s in sums]
        self.get_logger().info(
            f'Gyro bias (°/s): x={self.gyro_bias[0]:.2f} '
            f'y={self.gyro_bias[1]:.2f} z={self.gyro_bias[2]:.2f}')

    # ── Main loop ─────────────────────────────────────────────────────────────

    def tick(self):
        if not self._ok:
            return
        try:
            (ax, ay, az), (gx, gy, gz) = self._read_raw()
        except Exception as e:
            self.get_logger().warn(f'I2C read failed: {e}', throttle_duration_sec=5.0)
            return

        # Remove gyro bias
        gx -= self.gyro_bias[0]
        gy -= self.gyro_bias[1]
        gz -= self.gyro_bias[2]

        # dt from wall clock
        now = self.get_clock().now()
        if self._last_t is None:
            dt = 1.0 / self.rate_hz
        else:
            dt = (now - self._last_t).nanoseconds * 1e-9
            if dt <= 0.0 or dt > 0.5:
                dt = 1.0 / self.rate_hz
        self._last_t = now

        # Roll/pitch from the gravity vector (accelerometer)
        roll_acc  = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        # Complementary filter: gyro integration corrected by accel
        gx_rad = math.radians(gx)
        gy_rad = math.radians(gy)
        self.roll  = self.alpha * (self.roll  + gx_rad * dt) + (1.0 - self.alpha) * roll_acc
        self.pitch = self.alpha * (self.pitch + gy_rad * dt) + (1.0 - self.alpha) * pitch_acc

        self._publish(now, gx, gy, gz, ax, ay, az)

    def _publish(self, stamp, gx, gy, gz, ax, ay, az):
        msg = Imu()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.frame_id

        # Orientation quaternion from roll/pitch (yaw = 0)
        cr = math.cos(self.roll * 0.5);  sr = math.sin(self.roll * 0.5)
        cp = math.cos(self.pitch * 0.5); sp = math.sin(self.pitch * 0.5)
        msg.orientation.w = cr * cp
        msg.orientation.x = sr * cp
        msg.orientation.y = cr * sp
        msg.orientation.z = -sr * sp

        msg.angular_velocity.x = math.radians(gx)
        msg.angular_velocity.y = math.radians(gy)
        msg.angular_velocity.z = math.radians(gz)

        msg.linear_acceleration.x = ax * G_MS2
        msg.linear_acceleration.y = ay * G_MS2
        msg.linear_acceleration.z = az * G_MS2

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
