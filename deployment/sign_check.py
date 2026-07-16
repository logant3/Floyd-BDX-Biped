"""
sign_check.py — Floyd joint sign verification
==============================================
Sets all motors to zero-torque (freely backdrivable) and walks you
through each joint one at a time. For each joint you physically move
it in the described direction, then press Enter. The script records
whether that direction reads as positive or negative on the hardware.

Compare the summary against IsaacLab to catch any sign mismatches
before running the policy.

Usage:
    python3 deployment/sign_check.py

Requirements:
    sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
"""

import atexit
import signal
import struct
import sys
import threading
import time

import can
import numpy as np

# ── CAN / motor config ────────────────────────────────────────────────────────

CAN_CHANNEL  = "can0"
HOST_ID      = 0xFD
MUX_ENABLE   = 0x03
MUX_CONTROL  = 0x01
MUX_DISABLE  = 0x04
MSG_FEEDBACK = 0x02

MOTOR_NAMES = {
    1: "right_ankle",
    2: "left_ankle",
    3: "right_knee",
    4: "left_knee",
    5: "right_hip_pitch",
    6: "left_hip_pitch",
    7: "right_hip_roll",
    8: "left_hip_roll",
}

MOTOR_TYPE_PARAMS = {
    "O2": {"P_MIN":-12.57,"P_MAX":12.57,"V_MIN":-44.0,"V_MAX":44.0,
           "T_MIN":-17.0,"T_MAX":17.0,"KP_MIN":0.0,"KP_MAX":500.0,"KD_MIN":0.0,"KD_MAX":5.0},
    "O3": {"P_MIN":-12.57,"P_MAX":12.57,"V_MIN":-20.0,"V_MAX":20.0,
           "T_MIN":-60.0,"T_MAX":60.0,"KP_MIN":0.0,"KP_MAX":5000.0,"KD_MIN":0.0,"KD_MAX":100.0},
}

MOTOR_ID_TO_TYPE = {
    1:"O2", 2:"O2", 3:"O3", 4:"O3", 5:"O3", 6:"O3", 7:"O3", 8:"O3",
}

# Light damping so joints resist flopping but are easily moved by hand
BACKDRIVE_KD = {
    1: 0.3,   # right_ankle  (O2)
    2: 0.3,   # left_ankle   (O2)
    3: 1.0,   # right_knee   (O3)
    4: 1.0,   # left_knee    (O3)
    5: 1.0,   # right_hip_pitch (O3)
    6: 1.0,   # left_hip_pitch  (O3)
    7: 1.0,   # right_hip_roll  (O3)
    8: 1.0,   # left_hip_roll   (O3)
}

# Test sequence: (motor_id, joint_name, physical instruction)
# Each instruction describes a clear, unambiguous physical movement.
# The sign you see is the hardware convention for that direction.
TEST_SEQUENCE = [
    (6,  "left_hip_pitch",
     "Swing the LEFT thigh FORWARD — knee moves in front of the robot body"),

    (5,  "right_hip_pitch",
     "Swing the RIGHT thigh FORWARD — knee moves in front of the robot body"),

    (8,  "left_hip_roll",
     "Push the LEFT thigh OUTWARD — leg moves away from centre of robot"),

    (7,  "right_hip_roll",
     "Push the RIGHT thigh OUTWARD — leg moves away from centre of robot"),

    (4,  "left_knee",
     "BEND the LEFT knee — foot moves toward the back/top of the robot"),

    (3,  "right_knee",
     "BEND the RIGHT knee — foot moves toward the back/top of the robot"),

    (2,  "left_ankle",
     "DORSIFLEX the LEFT ankle — toes pull toward the shin (foot tips up)"),

    (1,  "right_ankle",
     "DORSIFLEX the RIGHT ankle — toes pull toward the shin (foot tips up)"),
]

# ── CAN helpers ───────────────────────────────────────────────────────────────

def _scale(val, v_min, v_max):
    return int(65535.0 * (np.clip(val, v_min, v_max) - v_min) / (v_max - v_min))

def _unscale(raw, v_min, v_max):
    return float(raw) / 65535.0 * (v_max - v_min) + v_min

def send_mit(bus, motor_id, pos, vel, kp, kd, torque):
    p = MOTOR_TYPE_PARAMS[MOTOR_ID_TO_TYPE[motor_id]]
    arb = (MUX_CONTROL << 24) | (_scale(torque, p["T_MIN"], p["T_MAX"]) << 8) | motor_id
    data = struct.pack(">HHHH",
                       _scale(pos,    p["P_MIN"], p["P_MAX"]),
                       _scale(vel,    p["V_MIN"], p["V_MAX"]),
                       _scale(kp,     p["KP_MIN"],p["KP_MAX"]),
                       _scale(kd,     p["KD_MIN"],p["KD_MAX"]))
    try:
        bus.send(can.Message(arbitration_id=arb, data=data,
                             is_extended_id=True, dlc=8))
    except can.CanOperationError:
        pass

def read_states(bus, states):
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
        states[motor_id] = pos

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

def send_backdrive_all(bus):
    """All motors: kp=0, light kd — freely movable but won't flop."""
    for mid in MOTOR_NAMES:
        send_mit(bus, mid, 0.0, 0.0, 0.0, BACKDRIVE_KD[mid], 0.0)

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  Floyd — Joint Sign Verification")
    print("=" * 60)
    print()
    print("All motors will be set to ZERO TORQUE with light damping.")
    print("You can move each joint by hand. Press Enter when in position.")
    print()
    input("Press Enter when you are ready to enable motors...")

    bus = can.interface.Bus(channel=CAN_CHANNEL, bustype="socketcan")
    atexit.register(lambda: (disable_all(bus), bus.shutdown()))

    _quit = [False]
    def _sigint(sig, frame):
        _quit[0] = True
    signal.signal(signal.SIGINT, _sigint)

    print("\nEnabling motors...")
    enable_all(bus)
    time.sleep(1.0)

    # Warm up — get initial positions
    states = {}
    for _ in range(50):
        send_backdrive_all(bus)
        time.sleep(0.02)
        read_states(bus, states)
    print("Motors enabled. All joints are now freely movable.\n")

    results = []

    for motor_id, joint_name, instruction in TEST_SEQUENCE:
        if _quit[0]:
            break

        print("─" * 60)
        print(f"  JOINT:  {joint_name}  (motor ID {motor_id})")
        print(f"  MOVE:   {instruction}")
        print()
        print("  Positions updating live. Press Enter once joint is in position.")
        print()

        # Background thread waits for Enter
        enter_event = threading.Event()
        def _wait_enter():
            try:
                input()
            except (EOFError, OSError):
                pass
            enter_event.set()
        waiter = threading.Thread(target=_wait_enter, daemon=True)
        waiter.start()

        # Live readout loop
        while not enter_event.is_set() and not _quit[0]:
            send_backdrive_all(bus)
            read_states(bus, states)

            current = states.get(motor_id, 0.0)
            others = "  ".join(
                f"M{mid}={states.get(mid, 0.0):+.2f}"
                for mid in sorted(MOTOR_NAMES) if mid != motor_id
            )
            sys.stdout.write(
                f"\r  >>> {joint_name}: {current:+.4f} rad <<<   "
                f"(others: {others})  "
            )
            sys.stdout.flush()
            time.sleep(0.05)

        print()

        recorded = states.get(motor_id, 0.0)

        if abs(recorded) < 0.05:
            verdict = "INCONCLUSIVE (< 0.05 rad) — move further and re-run"
        elif recorded > 0:
            verdict = "POSITIVE  → described direction = POSITIVE on hardware"
        else:
            verdict = "NEGATIVE  → described direction = NEGATIVE on hardware"

        print(f"\n  Recorded: {recorded:+.4f} rad")
        print(f"  Result:   {verdict}")
        print()

        results.append((motor_id, joint_name, recorded, instruction))

        # Brief pause before next joint
        for _ in range(30):
            if _quit[0]:
                break
            send_backdrive_all(bus)
            read_states(bus, states)
            time.sleep(0.02)

    # ── Summary ───────────────────────────────────────────────────────────────
    print("=" * 60)
    print("  SUMMARY — Hardware sign conventions")
    print("=" * 60)
    print(f"  {'Joint':<22s} {'Pos':>8s}  {'Described direction'}")
    print(f"  {'-'*22} {'-'*8}  {'-'*30}")
    for motor_id, joint_name, pos, instruction in results:
        sign = "POSITIVE" if pos > 0.05 else ("NEGATIVE" if pos < -0.05 else "ZERO??")
        direction = instruction.split("—")[0].strip()
        print(f"  {joint_name:<22s} {pos:>+8.4f}  {direction} = {sign}")

    print()
    print("Paste this output and we'll identify any sign flips needed.")

    disable_all(bus)
    bus.shutdown()
    print("\nDone.")


if __name__ == "__main__":
    main()
