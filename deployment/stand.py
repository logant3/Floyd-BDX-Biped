"""
stand.py — Floyd standing deployment
=====================================
Runs the trained policy with zero velocity commands. Use this to verify
the policy can balance before attempting any walking test.

Sequence:
  1. Connects to CAN and IMU
  2. Enables all 8 motors and interpolates to zero pose over 2s (standup gains)
  3. Holds at zero — press Enter to activate policy
  4. Policy runs with fall detection and joint-limit safety
  5. Ctrl+C to disable motors and exit

Run on Jetson:
    python3 deployment/stand.py --model path/to/policy.onnx

Requirements:
    pip install onnxruntime smbus2 python-can numpy
    sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
"""

import argparse
import atexit
import math
import os
import signal
import struct
import sys
import threading
import time

import can
import numpy as np
import onnxruntime as ort

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from imu_ism330dhcx import ISM330DHCXImu

# ──────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ──────────────────────────────────────────────────────────────────────────────

CAN_CHANNEL  = "can0"
HOST_ID      = 0xFD

# Motor ID → joint name (from motor allocation.pdf)
MOTOR_NAMES = {
    1: "right_ankle",    2: "left_ankle",
    3: "right_knee",     4: "left_knee",
    5: "right_hip_pitch",6: "left_hip_pitch",
    7: "right_hip_roll", 8: "left_hip_roll",
}

# IsaacLab alphabetical joint order → motor ID
# (obs and action vectors use this ordering)
JOINT_ORDER = [
    ("left_ankle",      2),   # index 0
    ("left_hip_pitch",  6),   # index 1
    ("left_hip_roll",   8),   # index 2
    ("left_knee",       4),   # index 3
    ("right_ankle",     1),   # index 4
    ("right_hip_pitch", 5),   # index 5
    ("right_hip_roll",  7),   # index 6
    ("right_knee",      3),   # index 7
]

# Motor type → CAN scaling params
MOTOR_TYPE_PARAMS = {
    "O2": {"P_MIN":-12.57,"P_MAX":12.57,"V_MIN":-44.0,"V_MAX":44.0,
           "T_MIN":-17.0,"T_MAX":17.0,"KP_MIN":0.0,"KP_MAX":500.0,"KD_MIN":0.0,"KD_MAX":5.0},
    "O3": {"P_MIN":-12.57,"P_MAX":12.57,"V_MIN":-20.0,"V_MAX":20.0,
           "T_MIN":-60.0,"T_MAX":60.0,"KP_MIN":0.0,"KP_MAX":5000.0,"KD_MIN":0.0,"KD_MAX":100.0},
}

MOTOR_ID_TO_TYPE = {
    1:"O2", 2:"O2", 3:"O3", 4:"O3", 5:"O3", 6:"O3", 7:"O3", 8:"O3",
}

# Policy gains (from rsl_rl_ppo_cfg / deployment config)
POLICY_KP = {1:16.581, 2:16.581, 3:78.957, 4:78.957, 5:78.957, 6:78.957, 7:78.957, 8:78.957}
POLICY_KD = {1:1.056,  2:1.056,  3:5.027,  4:5.027,  5:5.027,  6:5.027,  7:5.027,  8:5.027}

# Standup gains — significantly stiffer than policy to hold firmly during move-to-zero.
# RS03 (hip/knee): KP=200, RS02 (ankle): KP=60  (matches Kayden's hardware backend)
STANDUP_KP = {1:60.0, 2:60.0, 3:200.0, 4:200.0, 5:200.0, 6:200.0, 7:200.0, 8:200.0}
STANDUP_KD = {mid: kd for mid, kd in POLICY_KD.items()}

ACTION_SCALE = 0.5
OBS_DIM      = 33

# Safety limits
# Enter damping mode (kp=0, kd=1) if robot tilts past this angle
FALL_ANGLE_DEG  = 45.0
FALL_THRESHOLD  = math.cos(math.radians(FALL_ANGLE_DEG))  # 0.707
# Enter damping mode if any joint target exceeds ±MAX_TARGET_RAD
MAX_TARGET_RAD  = 1.5

# IMU — Floyd hardware defaults (Qwiic, bus 7, SA0=VCC → 0x6B)
FLOYD_MOUNTING_ROTATION = np.array([
    [ 0.102,  0.034, -0.994],
    [ 0.034,  0.999,  0.038],
    [ 0.994, -0.038,  0.101],
])

# CAN protocol
MUX_ENABLE   = 0x03
MUX_CONTROL  = 0x01
MUX_DISABLE  = 0x04
MSG_FEEDBACK = 0x02

# ──────────────────────────────────────────────────────────────────────────────
# CAN helpers
# ──────────────────────────────────────────────────────────────────────────────

def _scale(val, v_min, v_max):
    return int(65535.0 * (np.clip(val, v_min, v_max) - v_min) / (v_max - v_min))

def _unscale(raw, v_min, v_max):
    return float(raw) / 65535.0 * (v_max - v_min) + v_min

def _wrap_angle(v):
    """Wrap angle to (-π, π] for display and observation use only.
    Never apply this to positions used as motor commands — commands must
    stay in the motor's own reference frame."""
    return (v + math.pi) % (2 * math.pi) - math.pi

def _nearest_zero(pos):
    """The equivalent of 0.0 rad closest to pos in the motor's frame.
    Handles single-turn encoder wrapping: a motor physically at -0.12 rad
    may report 6.16 rad (≈ 2π - 0.12). Its standup target is 2π, not 0.0."""
    candidates = [0.0, 2 * math.pi, -2 * math.pi]
    return min(candidates, key=lambda c: abs(pos - c))

def send_mit(bus, motor_id, pos, vel, kp, kd, torque):
    p = MOTOR_TYPE_PARAMS[MOTOR_ID_TO_TYPE[motor_id]]
    a = _scale(pos,    p["P_MIN"], p["P_MAX"])
    v = _scale(vel,    p["V_MIN"], p["V_MAX"])
    k = _scale(kp,     p["KP_MIN"],p["KP_MAX"])
    d = _scale(kd,     p["KD_MIN"],p["KD_MAX"])
    t = _scale(torque, p["T_MIN"], p["T_MAX"])
    arb = (MUX_CONTROL << 24) | (t << 8) | motor_id
    try:
        bus.send(can.Message(arbitration_id=arb,
                             data=struct.pack(">HHHH", a, v, k, d),
                             is_extended_id=True, dlc=8))
    except can.CanOperationError:
        pass

def enable_all(bus):
    for mid in MOTOR_NAMES:
        bus.send(can.Message(arbitration_id=(MUX_ENABLE<<24)|(HOST_ID<<8)|mid,
                             is_extended_id=True, dlc=8))

def disable_all(bus):
    for mid in MOTOR_NAMES:
        try:
            bus.send(can.Message(arbitration_id=(MUX_DISABLE<<24)|(HOST_ID<<8)|mid,
                                 is_extended_id=True, dlc=8))
        except Exception:
            pass

def drain(bus):
    while bus.recv(timeout=0) is not None:
        pass

def flush_bus(bus):
    """Drain with a small timeout to clear any buffered stale frames."""
    while True:
        msg = bus.recv(timeout=0.01)
        if msg is None:
            break

def read_motor_states(bus, states):
    """Parse feedback frames into states dict {motor_id: (pos_rad, vel_rps)}."""
    while True:
        msg = bus.recv(timeout=0)
        if msg is None:
            break
        if msg.is_error_frame or len(msg.data) < 8:
            continue
        msg_type = (msg.arbitration_id & 0x1F000000) >> 24
        motor_id = (msg.arbitration_id & 0xFF00) >> 8
        if msg_type != MSG_FEEDBACK or motor_id not in MOTOR_NAMES:
            continue
        p = MOTOR_TYPE_PARAMS[MOTOR_ID_TO_TYPE[motor_id]]
        pos = _unscale(struct.unpack(">H", msg.data[0:2])[0], p["P_MIN"], p["P_MAX"])
        vel = _unscale(struct.unpack(">H", msg.data[2:4])[0], p["V_MIN"], p["V_MAX"])
        states[motor_id] = (pos, vel)

def send_damping(bus, motor_states):
    """Damping mode: kp=0, kd=1 at current position — lets robot fall safely."""
    for mid in MOTOR_NAMES:
        pos = motor_states.get(mid, (0.0, 0.0))[0]
        send_mit(bus, mid, pos, 0.0, 0.0, 1.0, 0.0)

# ──────────────────────────────────────────────────────────────────────────────
# Obs builder
# ──────────────────────────────────────────────────────────────────────────────

def build_obs(imu, states, prev_actions):
    """
    Assembles the 33-dim observation vector matching IsaacLab training order
    (velocity_env_cfg.py with base_lin_vel=None, height_scan=None):
      base_ang_vel ×0.2 (3) | projected_gravity (3) | velocity_commands (3) |
      joint_pos (8)          | joint_vel ×0.05 (8)   | actions (8)
    Velocity command is fixed at [0, 0, 0] for standing.
    """
    imu_data  = imu.get_latest_data()
    ang_vel   = imu_data["gyro"] * 0.2
    proj_grav = imu_data["projected_gravity"]

    joint_pos = np.zeros(8, dtype=np.float32)
    joint_vel = np.zeros(8, dtype=np.float32)
    for i, (_, motor_id) in enumerate(JOINT_ORDER):
        if motor_id in states:
            raw_pos, vel = states[motor_id]
            joint_pos[i] = _wrap_angle(raw_pos)  # wrap to (-π, π] to match IsaacLab training range
            joint_vel[i] = vel

    joint_vel_scaled = joint_vel * 0.05
    vel_cmd = np.zeros(3, dtype=np.float32)  # stand still

    obs = np.concatenate([
        ang_vel.astype(np.float32),
        proj_grav.astype(np.float32),
        vel_cmd,
        joint_pos,
        joint_vel_scaled,
        prev_actions,
    ])
    assert obs.shape[0] == OBS_DIM, f"obs dim mismatch: {obs.shape[0]} != {OBS_DIM}"
    return obs

# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model",    required=True,  help="Path to policy.onnx")
    parser.add_argument("--i2c_bus", type=int,   default=7,     help="I2C bus for IMU (default 7)")
    parser.add_argument("--ctrl_hz", type=float, default=400.0, help="CAN loop rate Hz (default 400)")
    parser.add_argument("--no_imu",  action="store_true",       help="Mock IMU (debug only)")
    parser.add_argument("--motors",  type=int,   nargs="+",     help="Only use these motor IDs (e.g. --motors 2 for left ankle only)")
    args = parser.parse_args()

    # Restrict active motors if --motors specified
    if args.motors:
        invalid = [m for m in args.motors if m not in MOTOR_NAMES]
        if invalid:
            print(f"[ERROR] Unknown motor IDs: {invalid}. Valid IDs: {list(MOTOR_NAMES.keys())}")
            return
        active_motors = {mid: MOTOR_NAMES[mid] for mid in args.motors}
        print(f"[TEST MODE] Only using motors: { {mid: MOTOR_NAMES[mid] for mid in args.motors} }")
    else:
        active_motors = MOTOR_NAMES

    CTRL_DT    = 1.0 / args.ctrl_hz
    DECIMATION = int(round(args.ctrl_hz / 50))  # policy runs at 50 Hz

    print("="*55)
    print("  Floyd — Standing Policy Deployment")
    print("="*55)
    print(f"  Model   : {args.model}")
    print(f"  Control : {args.ctrl_hz:.0f} Hz  |  Policy: {args.ctrl_hz/DECIMATION:.0f} Hz")
    print(f"  IMU     : {'MOCKED' if args.no_imu else f'I2C bus {args.i2c_bus}'}")
    print(f"  Safety  : fall>{FALL_ANGLE_DEG:.0f}deg  |  max_target={MAX_TARGET_RAD} rad")
    print()

    # ── Load policy ──────────────────────────────────────────────────────────
    print("Loading ONNX policy...")
    session    = ort.InferenceSession(args.model)
    input_name = session.get_inputs()[0].name
    print("  Policy loaded OK")

    # ── IMU ──────────────────────────────────────────────────────────────────
    if args.no_imu:
        class MockIMU:
            def get_latest_data(self):
                return {"gyro": np.zeros(3), "projected_gravity": np.array([0.,0.,-1.])}
            def stop(self): pass
        imu = MockIMU()
        print("  IMU: using mock (--no_imu)")
    else:
        imu = ISM330DHCXImu(i2c_bus=args.i2c_bus, i2c_addr=0x6B,
                             mounting_rotation=FLOYD_MOUNTING_ROTATION)
        if not imu.start():
            print("[FATAL] IMU failed to start. Use --no_imu to skip.")
            return

    # ── CAN bus ──────────────────────────────────────────────────────────────
    print(f"Opening {CAN_CHANNEL}...")
    bus = can.interface.Bus(channel=CAN_CHANNEL, bustype="socketcan")
    atexit.register(lambda: (disable_all(bus), bus.shutdown()))

    # ── Interrupt handling ────────────────────────────────────────────────────
    _quit = [False]
    def _sigint(sig, frame):
        _quit[0] = True
    signal.signal(signal.SIGINT, _sigint)

    # ── Enable motors ─────────────────────────────────────────────────────────
    print("\n[1/3] Enabling motors...")
    enable_all(bus)
    time.sleep(1.0)

    # Poll to get initial positions
    motor_states = {}
    for _ in range(50):
        for mid in active_motors:
            send_mit(bus, mid, 0.0, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.02)
        read_motor_states(bus, motor_states)

    # ── Verify all motors responded ──────────────────────────────────────────
    missing = [mid for mid in active_motors if mid not in motor_states]
    if missing:
        print(f"\n[ABORT] Motors not responding after 50 poll cycles:")
        for mid in missing:
            print(f"         Motor {mid} ({active_motors[mid]}): no CAN frames received")
        print("         Check CAN connection and motor power before retrying.")
        disable_all(bus)
        imu.stop()
        bus.shutdown()
        return

    start_pos = {}
    for mid in active_motors:
        raw = motor_states[mid][0]
        start_pos[mid] = raw
        wrapped = _wrap_angle(raw)
        print(f"   Motor {mid} ({active_motors[mid]:<18s}): start pos = {raw:.3f} rad  (wrapped: {wrapped:+.3f})")

    # Standup targets: nearest equivalent of 0 in each motor's raw frame.
    # e.g. a motor at 6.16 rad targets 2π (0.12 rad movement), not 0.0 (6.16 rad movement).
    standup_targets = {mid: _nearest_zero(start_pos[mid]) for mid in active_motors}

    # ── Encoder sanity check ──────────────────────────────────────────────────
    print()
    wrapped_pos = {mid: _wrap_angle(v) for mid, v in start_pos.items()}
    all_near_zero = all(abs(v) < 0.05 for v in wrapped_pos.values())
    out_of_range  = {mid: v for mid, v in wrapped_pos.items() if abs(v) > 2.0}

    if all_near_zero:
        print("[WARNING] All encoders read near 0.0 rad.")
        print("          Either motors zeroed at power-on, or robot was powered up in standing position.")
        print("          If legs were NOT in the standing position when powered on, readings are WRONG.")
        print()

    if out_of_range:
        print("[ABORT] Joint(s) outside ±2.0 rad — encoder error or bad mounting:")
        for mid, v in out_of_range.items():
            print(f"         Motor {mid} ({active_motors[mid]}): {v:+.3f} rad")
        print()
        disable_all(bus)
        imu.stop()
        bus.shutdown()
        return

    # Hold at zero torque while waiting — motors stay enabled but NO movement
    standup_event = threading.Event()
    def _wait_standup():
        try:
            input("\n[2/3] Press Enter to begin standup (motors will move to zero pose)...")
        except (EOFError, OSError):
            pass
        standup_event.set()
    threading.Thread(target=_wait_standup, daemon=True).start()

    while not standup_event.is_set() and not _quit[0]:
        drain(bus)
        for mid in active_motors:
            send_mit(bus, mid, 0.0, 0.0, 0.0, 0.0, 0.0)
        read_motor_states(bus, motor_states)
        time.sleep(0.02)

    if _quit[0]:
        print("\nAborted before standup.")
        disable_all(bus)
        imu.stop()
        bus.shutdown()
        return

    # ── Standup: interpolate to zero over 2s with KP ramp ────────────────────
    # KP starts very low (soft/compliant) and ramps to full standup KP as the
    # motor approaches its target. This prevents large torque spikes if the
    # starting position is slightly off.
    STANDUP_KP_START = 5.0   # gentle initial gain for all motors
    STANDUP_KD_START = 0.2   # gentle initial damping
    print("\n  Moving to zero pose (2s)...")
    STANDUP_STEPS = int(2.0 / 0.02)
    for step in range(STANDUP_STEPS + 1):
        if _quit[0]:
            break
        alpha = step / STANDUP_STEPS
        drain(bus)
        for mid in active_motors:
            target = (1.0 - alpha) * start_pos[mid] + alpha * standup_targets[mid]
            kp = STANDUP_KP_START + alpha * (STANDUP_KP[mid] - STANDUP_KP_START)
            kd = STANDUP_KD_START + alpha * (STANDUP_KD[mid] - STANDUP_KD_START)
            send_mit(bus, mid, target, 0.0, kp, kd, 0.0)
        read_motor_states(bus, motor_states)
        time.sleep(0.02)

    if _quit[0]:
        print("\nAborted during standup.")
        disable_all(bus)
        imu.stop()
        bus.shutdown()
        return

    print("  Zero pose reached.")
    print("\n  Place Floyd on the ground if not already.")

    # ── Hold at zero while waiting for Enter ──────────────────────────────────
    # Background thread waits for Enter; main loop keeps sending standup commands
    # so motors stay active and Floyd doesn't go limp.
    enter_event = threading.Event()
    def _wait_for_enter():
        try:
            input("\n[3/3] Press Enter to activate policy (Ctrl+C to abort)...")
        except (EOFError, OSError):
            pass
        enter_event.set()

    waiter = threading.Thread(target=_wait_for_enter, daemon=True)
    waiter.start()

    while not enter_event.is_set() and not _quit[0]:
        drain(bus)
        for mid in active_motors:
            send_mit(bus, mid, standup_targets[mid], 0.0, STANDUP_KP[mid], STANDUP_KD[mid], 0.0)
        read_motor_states(bus, motor_states)
        time.sleep(0.02)

    if _quit[0]:
        print("\nAborted before policy activation.")
        disable_all(bus)
        imu.stop()
        bus.shutdown()
        return

    # ── Flush CAN buffer before policy takes over ─────────────────────────────
    print("\n  Flushing CAN buffer...")
    flush_bus(bus)

    # ── Policy loop ──────────────────────────────────────────────────────────
    print("  Policy active. Ctrl+C to stop.\n")

    prev_actions   = np.zeros(8, dtype=np.float32)
    actions        = np.zeros(8, dtype=np.float32)
    it             = 0
    last_print     = time.time()
    t_last         = time.perf_counter()
    in_damping     = False

    while not _quit[0]:
        # ── Safety: fall detection ────────────────────────────────────────────
        if not in_damping:
            imu_data = imu.get_latest_data()
            pg_z = float(imu_data["projected_gravity"][2])
            # upright = -1.0; fallen past 45° means pg_z > -cos(45°) = -0.707
            if pg_z > -FALL_THRESHOLD:
                print(f"\n[SAFETY] Fall detected! proj_grav_z={pg_z:.3f} "
                      f"(threshold={-FALL_THRESHOLD:.3f}). Entering damping mode.")
                in_damping = True

        # ── Damping mode: let the robot fall safely ───────────────────────────
        if in_damping:
            drain(bus)
            send_damping(bus, motor_states)
            read_motor_states(bus, motor_states)
            elapsed = time.perf_counter() - t_last
            sleep = CTRL_DT - elapsed
            if sleep > 0:
                time.sleep(sleep)
            t_last = time.perf_counter()
            it += 1
            continue

        # ── Policy inference at 50 Hz ─────────────────────────────────────────
        if it % DECIMATION == 0:
            obs     = build_obs(imu, motor_states, prev_actions)
            actions = session.run(None, {input_name: obs.reshape(1, -1)})[0][0]
            prev_actions = actions.copy()

        # ── Safety: clamp targets, enter damping if wildly out of range ───────
        drain(bus)
        for i, (_, motor_id) in enumerate(JOINT_ORDER):
            if motor_id not in active_motors:
                continue
            # Policy action is relative to standing pose (0.0 in IsaacLab).
            # On hardware, standing pose = standup_targets[motor_id] in the motor's
            # raw encoder frame (may be 0.0 or ±2π depending on power-on position).
            # Add the frame offset so the motor stays near its physical zero.
            frame_offset = standup_targets[motor_id]
            target = frame_offset + float(actions[i]) * ACTION_SCALE
            delta = float(actions[i]) * ACTION_SCALE  # for safety check
            if abs(delta) > MAX_TARGET_RAD:
                jname = JOINT_ORDER[i][0]
                print(f"\n[SAFETY] Target for {jname} = {delta:.3f} rad "
                      f"exceeds ±{MAX_TARGET_RAD} rad. Entering damping mode.")
                in_damping = True
                break
            send_mit(bus, motor_id, target, 0.0, POLICY_KP[motor_id], POLICY_KD[motor_id], 0.0)

        read_motor_states(bus, motor_states)

        # ── Status print every second ─────────────────────────────────────────
        now = time.time()
        if now - last_print >= 1.0:
            imu_data = imu.get_latest_data()
            pg = imu_data["projected_gravity"]
            print(f"  iter {it:6d} | proj_grav [{pg[0]:+.3f}, {pg[1]:+.3f}, {pg[2]:+.3f}] "
                  f"| actions max {np.abs(actions).max():.3f}")
            last_print = now

        # ── Pace to ctrl_hz ───────────────────────────────────────────────────
        it += 1
        elapsed = time.perf_counter() - t_last
        sleep   = CTRL_DT - elapsed
        if sleep > 0:
            time.sleep(sleep)
        t_last = time.perf_counter()

    print("\nCtrl+C — disabling motors.")
    disable_all(bus)
    time.sleep(0.1)
    imu.stop()
    bus.shutdown()
    print("Done.")


if __name__ == "__main__":
    import traceback
    try:
        main()
    except Exception as e:
        print(f"\n[CRASH] Unhandled exception: {e}")
        traceback.print_exc()
        print("[CRASH] Motors will be disabled by atexit handler.")
        sys.exit(1)
