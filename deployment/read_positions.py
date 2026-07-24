"""
read_positions.py — Floyd motor position reader
================================================
Reads and displays the current position of every motor.
NO movement possible — uses ReadParam (comm type 17) which queries
the motor's position register without entering torque control mode.
This is the same robstride library used by set_perm_zeros.py.

Run on Jetson:
    python3 deployment/read_positions.py

Requirements:
    sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
"""

import math
import time
import can
import robstride.client

# ── Config ───────────────────────────────────────────────────────────────────
CAN_CHANNEL = "can0"

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

# ── Helpers ──────────────────────────────────────────────────────────────────

def wrap(v):
    """Wrap to (-π, π] — what the policy sees after _wrap_angle."""
    return (v + math.pi) % (2 * math.pi) - math.pi

# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("Opening can0...")
    bus = can.interface.Bus(interface="socketcan", channel=CAN_CHANNEL)
    client = robstride.client.Client(bus)

    print("Reading motor positions (no movement, no enable)...\n")

    results = {}
    for mid in sorted(MOTOR_NAMES):
        try:
            # ReadParam (comm type 17) reads the mechanical position register
            # directly without enabling the motor or entering torque control.
            pos = client.read_param(mid, "mechpos")
            results[mid] = pos
        except Exception as e:
            results[mid] = None
            print(f"  Motor {mid} ({MOTOR_NAMES[mid]}): ERROR — {e}")
        time.sleep(0.05)

    bus.shutdown()

    print()
    print("=" * 60)
    print("  Floyd Motor Positions")
    print("=" * 60)
    print(f"  {'ID':<4} {'Joint':<18} {'Raw (rad)':>10}  {'Wrapped':>10}  Status")
    print(f"  {'-'*4} {'-'*18} {'-'*10}  {'-'*10}  {'-'*8}")

    for mid in sorted(MOTOR_NAMES):
        name = MOTOR_NAMES[mid]
        pos  = results[mid]
        if pos is None:
            print(f"  {mid:<4} {name:<18} {'---':>10}  {'---':>10}  NO RESPONSE")
        else:
            w = wrap(pos)
            status = "OK" if abs(w) < 0.1 else f"OFF {w:+.3f} rad"
            print(f"  {mid:<4} {name:<18} {pos:>+10.3f}  {w:>+10.3f}  {status}")

    print("=" * 60)
    print()
    print("'Wrapped' is what the policy sees. Should be near 0.0 rad")
    print("for all joints if zeroed correctly in the standing pose.")

if __name__ == "__main__":
    main()
