"""
ISM330DHCX IMU driver for Floyd deployment.

Drop-in replacement for Kayden's MTi_Serial_IMU. Uses the SparkFun 9DoF
ISM330DHCX + MMC5983MA breakout over Qwiic (I2C). Only the ISM330DHCX
(accel + gyro) is needed — the magnetometer is not used.

A Mahony complementary filter fuses accel (low-freq gravity reference) and
gyro (high-freq integration) to produce the projected gravity vector that
the policy expects.

Interface (identical to MTi_Serial_IMU):
    imu = ISM330DHCXImu(i2c_bus=1)
    imu.start()
    data = imu.get_latest_data()   # {"gyro": np.ndarray[3], "projected_gravity": np.ndarray[3]}
    imu.stop()

Jetson setup:
    pip install smbus2
    sudo i2cdetect -l                   # find which bus the Qwiic header is on
    sudo i2cdetect -y <bus>             # should see 0x6A

    # ISM330DHCX I2C address:
    #   SA0 pin = GND (default Qwiic) → 0x6A
    #   SA0 pin = VCC                 → 0x6B

IMPORTANT — IMU mounting orientation:
    The policy was trained with gravity = [0, 0, -1] in IsaacLab world frame.
    If the IMU is not mounted with its +Z axis pointing up on the robot, the
    projected_gravity vector will be wrong. Use the `mounting_rotation` parameter
    or physically orient the board Z-up. Verify by printing projected_gravity
    when the robot is upright — it should be close to [0.0, 0.0, -1.0].
"""

from __future__ import annotations

import math
import struct
import threading
import time
from typing import Optional

import numpy as np

try:
    import smbus2
    _HAS_SMBUS = True
except ImportError:
    _HAS_SMBUS = False


# ---------------------------------------------------------------------------
# ISM330DHCX register map
# ---------------------------------------------------------------------------

_WHO_AM_I_REG  = 0x0F   # Should read 0x6B
_CTRL1_XL      = 0x10   # Accel: ODR + full-scale
_CTRL2_G       = 0x11   # Gyro:  ODR + full-scale
_CTRL3_C       = 0x12   # BDU, auto-increment
_STATUS_REG    = 0x1E
_OUTX_L_G      = 0x22   # Gyro  X low byte (6 bytes total)
_OUTX_L_A      = 0x28   # Accel X low byte (6 bytes total)

_WHO_AM_I_VAL  = 0x6B

# ODR → 4-bit code (bits [7:4] of CTRL1/CTRL2)
_ODR_CODES = {
    12:  0x1,
    26:  0x2,
    52:  0x3,
    104: 0x4,
    208: 0x5,
    416: 0x6,
    833: 0x7,
}

# Accel full-scale: ±4g  (bits [3:2] = 0b10)
# Sensitivity: 0.122 mg/LSB → 0.122e-3 * 9.80665 m/s²/LSB
_ACCEL_FS_BITS   = 0b10
_ACCEL_SCALE_MPS2 = 0.122e-3 * 9.80665

# Gyro full-scale: ±2000 dps  (bits [3:2] = 0b11)
# Sensitivity: 70 mdps/LSB → 70e-3 * π/180 rad/s/LSB
_GYRO_FS_BITS   = 0b11
_GYRO_SCALE_RPS = 70e-3 * (math.pi / 180.0)


# ---------------------------------------------------------------------------
# Mahony complementary filter
# ---------------------------------------------------------------------------

class _MahonyFilter:
    """
    Lightweight Mahony filter — fuses accel and gyro into a quaternion,
    then derives the projected gravity vector.

    kp: proportional gain (accel correction strength)
    ki: integral gain    (gyro bias correction, keep small)
    """

    def __init__(self, kp: float = 2.0, ki: float = 0.005):
        self._kp = kp
        self._ki = ki
        self._q  = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)  # [w, x, y, z]
        self._integral = np.zeros(3, dtype=np.float64)

    def update(self, gx: float, gy: float, gz: float,
               ax: float, ay: float, az: float,
               dt: float) -> np.ndarray:
        """
        Update filter and return projected_gravity in body frame ([0,0,-1] rotated
        into body frame by current orientation estimate).
        """
        qw, qx, qy, qz = self._q

        # Normalise accel — if near-zero it's noise, skip correction
        a_norm = math.sqrt(ax*ax + ay*ay + az*az)
        if a_norm > 1e-6:
            ax /= a_norm
            ay /= a_norm
            az /= a_norm

            # Estimated gravity direction in body frame from current quaternion
            # = lower row of rotation matrix R = q.as_matrix() transposed
            vx = 2.0 * (qx*qz - qw*qy)
            vy = 2.0 * (qw*qx + qy*qz)
            vz = qw*qw - qx*qx - qy*qy + qz*qz

            # Error = cross(measured_gravity, estimated_gravity)
            ex = ay*vz - az*vy
            ey = az*vx - ax*vz
            ez = ax*vy - ay*vx

            # Integral feedback
            self._integral += self._ki * np.array([ex, ey, ez]) * dt
            gx += self._kp * ex + self._integral[0]
            gy += self._kp * ey + self._integral[1]
            gz += self._kp * ez + self._integral[2]

        # Integrate quaternion kinematics
        half_dt = 0.5 * dt
        dqw = (-qx*gx - qy*gy - qz*gz) * half_dt
        dqx = ( qw*gx + qy*gz - qz*gy) * half_dt
        dqy = ( qw*gy - qx*gz + qz*gx) * half_dt
        dqz = ( qw*gz + qx*gy - qy*gx) * half_dt

        self._q = np.array([qw+dqw, qx+dqx, qy+dqy, qz+dqz])
        self._q /= np.linalg.norm(self._q)

        # Projected gravity = R^T @ [0, 0, -1] = -(lower row of R)
        qw, qx, qy, qz = self._q
        proj_grav = np.array([
            -2.0 * (qx*qz - qw*qy),
            -2.0 * (qw*qx + qy*qz),
            -(qw*qw - qx*qx - qy*qy + qz*qz),
        ])
        return proj_grav


# ---------------------------------------------------------------------------
# ISM330DHCX IMU class
# ---------------------------------------------------------------------------

class ISM330DHCXImu:
    """
    SparkFun ISM330DHCX (Qwiic, I2C) driver for Floyd deployment.

    Args:
        i2c_bus:   Linux I2C bus number (e.g. 1 → /dev/i2c-1).
                   Find yours with: sudo i2cdetect -l
        i2c_addr:  0x6A (SA0=GND, Qwiic default) or 0x6B (SA0=VCC).
        odr_hz:    Output data rate. Choose from 52, 104, 208, 416, 833 Hz.
                   208 Hz is more than enough for a 50 Hz policy loop.
        mahony_kp: Proportional gain for Mahony filter (higher = faster
                   convergence but more sensitive to accel noise).
        mahony_ki: Integral gain (gyro bias correction). Keep small.
        mounting_rotation: Optional 3×3 rotation matrix applied to raw sensor
                   readings to correct for physical mounting orientation.
                   Identity (None) assumes sensor Z-axis points up.
    """

    def __init__(
        self,
        i2c_bus: int = 1,
        i2c_addr: int = 0x6A,
        odr_hz: int = 208,
        mahony_kp: float = 2.0,
        mahony_ki: float = 0.005,
        mounting_rotation: Optional[np.ndarray] = None,
    ):
        if not _HAS_SMBUS:
            raise ImportError("smbus2 is required: pip install smbus2")

        self._bus_num  = i2c_bus
        self._addr     = i2c_addr
        self._odr_hz   = odr_hz
        self._filter   = _MahonyFilter(kp=mahony_kp, ki=mahony_ki)
        self._R_mount  = mounting_rotation  # None = identity

        self._bus: Optional[smbus2.SMBus] = None
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._running = False

        self._latest = {
            "gyro":               np.zeros(3, dtype=np.float32),
            "projected_gravity":  np.array([0.0, 0.0, -1.0], dtype=np.float32),
        }

    # ------------------------------------------------------------------ I2C

    def _write_reg(self, reg: int, val: int) -> None:
        self._bus.write_byte_data(self._addr, reg, val)

    def _read_bytes(self, reg: int, n: int) -> bytes:
        return bytes(self._bus.read_i2c_block_data(self._addr, reg, n))

    # --------------------------------------------------------------- Config

    def _configure(self) -> None:
        # Enable block-data update and register auto-increment
        self._write_reg(_CTRL3_C, 0x44)
        time.sleep(0.01)

        odr_code = _ODR_CODES.get(self._odr_hz, _ODR_CODES[208])

        # CTRL1_XL: ODR[7:4] | FS_XL[3:2] | 0[1:0]
        self._write_reg(_CTRL1_XL, (odr_code << 4) | (_ACCEL_FS_BITS << 2))

        # CTRL2_G:  ODR[7:4] | FS_G[3:2]  | 0[1:0]
        self._write_reg(_CTRL2_G,  (odr_code << 4) | (_GYRO_FS_BITS << 2))

        time.sleep(0.1)  # Wait for sensor to stabilise

    # ------------------------------------------------------------ Read loop

    def _read_raw(self):
        """Returns (gx, gy, gz, ax, ay, az) in rad/s and m/s²."""
        raw_g = self._read_bytes(_OUTX_L_G, 6)
        raw_a = self._read_bytes(_OUTX_L_A, 6)

        gx, gy, gz = (
            struct.unpack("<h", raw_g[i:i+2])[0] * _GYRO_SCALE_RPS
            for i in (0, 2, 4)
        )
        ax, ay, az = (
            struct.unpack("<h", raw_a[i:i+2])[0] * _ACCEL_SCALE_MPS2
            for i in (0, 2, 4)
        )
        return gx, gy, gz, ax, ay, az

    def _apply_mounting_rotation(self, v: np.ndarray) -> np.ndarray:
        if self._R_mount is None:
            return v
        return self._R_mount @ v

    def _read_loop(self) -> None:
        dt_target = 1.0 / self._odr_hz
        t_last = time.perf_counter()

        while self._running:
            try:
                gx, gy, gz, ax, ay, az = self._read_raw()

                # Apply mounting orientation correction
                g_vec = self._apply_mounting_rotation(np.array([gx, gy, gz]))
                a_vec = self._apply_mounting_rotation(np.array([ax, ay, az]))

                now = time.perf_counter()
                dt  = now - t_last
                t_last = now

                proj_grav = self._filter.update(
                    g_vec[0], g_vec[1], g_vec[2],
                    a_vec[0], a_vec[1], a_vec[2],
                    dt,
                )

                with self._lock:
                    self._latest["gyro"]              = g_vec.astype(np.float32)
                    self._latest["projected_gravity"] = proj_grav.astype(np.float32)

                elapsed = time.perf_counter() - now
                sleep_t = dt_target - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)

            except Exception:
                # Transient I2C errors are recoverable; log silently
                time.sleep(0.001)

    # ---------------------------------------------------------- Public API

    def start(self) -> bool:
        """
        Open the I2C bus, verify the chip, configure it, and start the
        background read thread.

        Returns True on success, False on failure.
        """
        try:
            self._bus = smbus2.SMBus(self._bus_num)
            who = self._bus.read_byte_data(self._addr, _WHO_AM_I_REG)
            if who != _WHO_AM_I_VAL:
                print(
                    f"[ISM330DHCX] WHO_AM_I = 0x{who:02X}, expected 0x{_WHO_AM_I_VAL:02X}. "
                    f"Check wiring or try --i2c_addr 0x6B."
                )
                return False
        except Exception as e:
            print(f"[ISM330DHCX] Failed to open I2C bus {self._bus_num} at 0x{self._addr:02X}: {e}")
            return False

        self._configure()
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        print(
            f"[ISM330DHCX] Ready — bus {self._bus_num}, "
            f"addr 0x{self._addr:02X}, {self._odr_hz} Hz, "
            f"±4g accel, ±2000 dps gyro, Mahony filter"
        )
        return True

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._bus is not None:
            self._bus.close()

    def get_latest_data(self) -> dict:
        """
        Returns the most recent IMU data.

        Keys:
            "gyro"              np.ndarray[3]  angular velocity (rad/s) in body frame
            "projected_gravity" np.ndarray[3]  gravity direction in body frame
                                               should be ~[0, 0, -1] when upright
        """
        with self._lock:
            return {
                "gyro":              self._latest["gyro"].copy(),
                "projected_gravity": self._latest["projected_gravity"].copy(),
            }


# ---------------------------------------------------------------------------
# Quick sanity check — run directly on the Jetson to verify wiring
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse

    # Floyd hardware defaults: Qwiic on bus 7, SA0=VCC → 0x6B
    # Mounting rotation corrects for physical IMU orientation on Floyd's front-right base plate.
    # Computed from measured proj_grav [-0.994, 0.038, -0.101] when robot upright.
    FLOYD_MOUNTING_ROTATION = np.array([
        [ 0.102,  0.034, -0.994],
        [ 0.034,  0.999,  0.038],
        [ 0.994, -0.038,  0.101],
    ])

    parser = argparse.ArgumentParser(description="ISM330DHCX IMU sanity check")
    parser.add_argument("--bus",  type=lambda x: int(x, 0), default=7,    help="I2C bus number (default 7)")
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=0x6B, help="I2C address (default 0x6B)")
    parser.add_argument("--no-rotation", action="store_true", help="Disable mounting rotation (raw sensor frame)")
    args = parser.parse_args()

    rotation = None if args.no_rotation else FLOYD_MOUNTING_ROTATION
    imu = ISM330DHCXImu(i2c_bus=args.bus, i2c_addr=args.addr, mounting_rotation=rotation)
    if not imu.start():
        raise SystemExit(1)

    print("\nRunning — hold robot upright. projected_gravity should converge to ~[0, 0, -1].")
    print("Press Ctrl+C to stop.\n")
    try:
        while True:
            d = imu.get_latest_data()
            g  = d["gyro"]
            pg = d["projected_gravity"]
            print(
                f"gyro: [{g[0]:+.3f}, {g[1]:+.3f}, {g[2]:+.3f}] rad/s  |  "
                f"proj_grav: [{pg[0]:+.4f}, {pg[1]:+.4f}, {pg[2]:+.4f}]",
                end="\r",
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        imu.stop()
        print("\nDone.")
