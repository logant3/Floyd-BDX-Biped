"""
collect_real.py — Single-motor bench identification data collection
====================================================================
Run on Jetson with ONE motor connected on can0.
Commands a step sequence and logs (time, q_cmd, q_actual, dq_actual, tau_actual)
to a CSV for comparison against the sim actuator model.

Usage:
    python3 deployment/actuator_ident/collect_real.py --motor-id 3
    python3 deployment/actuator_ident/collect_real.py --motor-id 1 --output rs02_bench.csv

Motor IDs:
    1 = right_ankle  (RS02)    2 = left_ankle   (RS02)
    3 = right_knee   (RS03)    4 = left_knee     (RS03)
    5 = right_hip_pitch (RS03) 6 = left_hip_pitch (RS03)
    7 = right_hip_roll  (RS03) 8 = left_hip_roll  (RS03)

Step sequence (each step held for HOLD_TIME seconds):
    0 → +A → 0 → -A → 0 → +2A → 0 → -2A → 0  (A = small step, 2A = large step)
Followed by a slow sinusoidal sweep to exercise the full gain curve.

Press Ctrl+C at any time — motor is disabled safely on exit.
"""

import argparse
import atexit
import csv
import math
import os
import signal
import struct
import sys
import time

import can

# ──────────────────────────────────────────────────────────────────────────────
# Config
# ──────────────────────────────────────────────────────────────────────────────

CAN_CHANNEL = "can0"
HOST_ID     = 0xFD
LOOP_HZ     = 200       # command + log rate (Hz)
HOLD_TIME   = 2.0       # seconds to hold each step
RAMP_TIME   = 3.0       # seconds to ramp from encoder position to zero

MUX_ENABLE   = 0x03
MUX_CONTROL  = 0x01
MUX_DISABLE  = 0x04
MSG_FEEDBACK = 0x02

MOTOR_TYPE_PARAMS = {
    "O2": {
        "P_MIN": -12.57, "P_MAX": 12.57,
        "V_MIN": -44.0,  "V_MAX": 44.0,
        "T_MIN": -17.0,  "T_MAX": 17.0,
        "KP_MIN": 0.0,   "KP_MAX": 500.0,
        "KD_MIN": 0.0,   "KD_MAX": 5.0,
        # IsaacLab config values
        "KP": 16.581, "KD": 1.056, "EFFORT_LIMIT": 17.0,
        "SMALL_STEP": 0.15, "LARGE_STEP": 0.30,
        "SINE_AMP": 0.25, "SINE_FREQ": 0.3,
    },
    "O3": {
        "P_MIN": -12.57, "P_MAX": 12.57,
        "V_MIN": -20.0,  "V_MAX": 20.0,
        "T_MIN": -60.0,  "T_MAX": 60.0,
        "KP_MIN": 0.0,   "KP_MAX": 5000.0,
        "KD_MIN": 0.0,   "KD_MAX": 100.0,
        # IsaacLab config values
        "KP": 78.957, "KD": 5.027, "EFFORT_LIMIT": 60.0,
        "SMALL_STEP": 0.20, "LARGE_STEP": 0.45,
        "SINE_AMP": 0.40, "SINE_FREQ": 0.3,
    },
}

MOTOR_ID_TO_TYPE = {
    1: "O2", 2: "O2",
    3: "O3", 4: "O3", 5: "O3", 6: "O3", 7: "O3", 8: "O3",
}

MOTOR_NAMES = {
    1: "right_ankle", 2: "left_ankle",
    3: "right_knee",  4: "left_knee",
    5: "right_hip_pitch", 6: "left_hip_pitch",
    7: "right_hip_roll",  8: "left_hip_roll",
}

# ──────────────────────────────────────────────────────────────────────────────
# CAN helpers
# ──────────────────────────────────────────────────────────────────────────────

def _scale(val, v_min, v_max):
    return int(65535.0 * (max(v_min, min(v_max, val)) - v_min) / (v_max - v_min))

def _unscale(raw, v_min, v_max):
    return float(raw) / 65535.0 * (v_max - v_min) + v_min

def enable_motor(bus, motor_id):
    bus.send(can.Message(
        arbitration_id=(MUX_ENABLE << 24) | (HOST_ID << 8) | motor_id,
        is_extended_id=True, dlc=8))
    time.sleep(0.1)

def disable_motor(bus, motor_id):
    try:
        bus.send(can.Message(
            arbitration_id=(MUX_DISABLE << 24) | (HOST_ID << 8) | motor_id,
            is_extended_id=True, dlc=8))
    except Exception:
        pass

def send_mit(bus, motor_id, pos, vel, kp, kd, torque_ff=0.0):
    p = MOTOR_TYPE_PARAMS[MOTOR_ID_TO_TYPE[motor_id]]
    a = _scale(pos,       p["P_MIN"], p["P_MAX"])
    v = _scale(vel,       p["V_MIN"], p["V_MAX"])
    k = _scale(kp,        p["KP_MIN"], p["KP_MAX"])
    d = _scale(kd,        p["KD_MIN"], p["KD_MAX"])
    t = _scale(torque_ff, p["T_MIN"],  p["T_MAX"])
    arb = (MUX_CONTROL << 24) | (t << 8) | motor_id
    bus.send(can.Message(
        arbitration_id=arb,
        data=struct.pack(">HHHH", a, v, k, d),
        is_extended_id=True, dlc=8))

def read_feedback(bus, motor_id):
    """
    Read and parse the latest feedback frame for the given motor.
    Returns (pos_rad, vel_rps, tau_nm) or None if no frame available.
    Feedback frame layout (big-endian uint16 each):
        bytes 0-1: position
        bytes 2-3: velocity
        bytes 4-5: torque
    """
    result = None
    for _ in range(20):  # drain up to 20 frames, keep the latest for this motor
        msg = bus.recv(timeout=0.002)
        if msg is None:
            break
        if msg.is_error_frame or len(msg.data) < 6:
            continue
        msg_type = (msg.arbitration_id & 0x1F000000) >> 24
        mid      = (msg.arbitration_id & 0xFF00) >> 8
        if msg_type != MSG_FEEDBACK or mid != motor_id:
            continue
        p = MOTOR_TYPE_PARAMS[MOTOR_ID_TO_TYPE[motor_id]]
        pos = _unscale(struct.unpack(">H", msg.data[0:2])[0], p["P_MIN"], p["P_MAX"])
        vel = _unscale(struct.unpack(">H", msg.data[2:4])[0], p["V_MIN"], p["V_MAX"])
        tau = _unscale(struct.unpack(">H", msg.data[4:6])[0], p["T_MIN"], p["T_MAX"])
        result = (pos, vel, tau)
    return result

# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def build_step_sequence(p):
    """
    Returns list of (q_cmd, duration_s) tuples.
    Small steps show the linear gain region; large steps push toward saturation.
    """
    A = p["SMALL_STEP"]
    B = p["LARGE_STEP"]
    hold = HOLD_TIME
    return [
        (0.0,  hold),
        ( A,   hold),
        (0.0,  hold),
        (-A,   hold),
        (0.0,  hold),
        ( B,   hold),
        (0.0,  hold),
        (-B,   hold),
        (0.0,  hold),
    ]

def main():
    parser = argparse.ArgumentParser(description="Bench actuator data collection")
    parser.add_argument("--motor-id",  type=int, required=True,
                        choices=list(MOTOR_NAMES.keys()),
                        help="Motor ID to test (1-8)")
    parser.add_argument("--output", type=str, default=None,
                        help="Output CSV path (default: auto-named by motor)")
    args = parser.parse_args()

    mid  = args.motor_id
    mtype = MOTOR_ID_TO_TYPE[mid]
    p    = MOTOR_TYPE_PARAMS[mtype]
    name = MOTOR_NAMES[mid]

    out_path = args.output or f"{name}_RS{mtype[1:]}02_bench.csv".replace("O2","RS02").replace("O3","RS03")
    # clean up the filename
    out_path = f"{name}_bench.csv"
    if args.output:
        out_path = args.output

    print(f"\n=== Floyd Actuator Bench Test ===")
    print(f"  Motor:  ID={mid}  name={name}  type=RS{mtype[1:]}")
    print(f"  Gains:  kp={p['KP']}  kd={p['KD']}  effort_limit=±{p['EFFORT_LIMIT']} Nm")
    print(f"  Output: {out_path}")
    print(f"\nEnsure motor is on bench, free to rotate, CAN connected.")
    input("Press Enter to start...\n")

    bus = can.interface.Bus(channel=CAN_CHANNEL, bustype="socketcan")
    shutdown_requested = False

    def shutdown(*_):
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    atexit.register(lambda: disable_motor(bus, mid))

    # ── Enable and get initial position ──────────────────────────────────────
    print("Enabling motor...")
    enable_motor(bus, mid)
    time.sleep(0.2)

    # Send a gentle zero-torque command and read back position
    send_mit(bus, mid, 0.0, 0.0, 0.0, p["KD"], 0.0)
    time.sleep(0.05)
    fb = read_feedback(bus, mid)
    if fb is None:
        print("ERROR: No feedback from motor. Check CAN connection and motor power.")
        disable_motor(bus, mid)
        sys.exit(1)

    start_pos, _, _ = fb
    print(f"Motor at position: {start_pos:.4f} rad")

    # ── Ramp to zero ─────────────────────────────────────────────────────────
    print(f"Ramping to zero over {RAMP_TIME:.0f}s...")
    dt      = 1.0 / LOOP_HZ
    steps   = int(RAMP_TIME * LOOP_HZ)
    t_ramp  = time.monotonic()

    for i in range(steps):
        if shutdown_requested:
            break
        alpha   = i / max(steps - 1, 1)
        q_cmd   = start_pos * (1.0 - alpha)  # ramp from start_pos → 0
        send_mit(bus, mid, q_cmd, 0.0, p["KP"], p["KD"], 0.0)
        read_feedback(bus, mid)  # drain; we don't need to log ramp data
        elapsed = time.monotonic() - t_ramp
        next_t  = (i + 1) * dt
        sleep_t = next_t - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

    print("At zero. Starting step sequence...")

    # ── Step sequence + logging ───────────────────────────────────────────────
    sequence  = build_step_sequence(p)
    log_rows  = []
    t0        = time.monotonic()

    for q_cmd, duration in sequence:
        if shutdown_requested:
            break
        seg_end = time.monotonic() + duration
        print(f"  q_cmd = {q_cmd:+.3f} rad  ({duration:.1f}s)")

        while time.monotonic() < seg_end:
            if shutdown_requested:
                break
            loop_start = time.monotonic()

            send_mit(bus, mid, q_cmd, 0.0, p["KP"], p["KD"], 0.0)
            fb = read_feedback(bus, mid)

            t_now = time.monotonic() - t0
            if fb is not None:
                pos, vel, tau = fb
                log_rows.append({
                    "t":         round(t_now, 5),
                    "q_cmd":     round(q_cmd, 6),
                    "q_actual":  round(pos, 6),
                    "dq_actual": round(vel, 6),
                    "tau_actual": round(tau, 6),
                })

            elapsed = time.monotonic() - loop_start
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    # ── Sinusoidal sweep ─────────────────────────────────────────────────────
    if not shutdown_requested:
        sine_dur  = 1.0 / p["SINE_FREQ"] * 4   # 4 full cycles
        amp       = p["SINE_AMP"]
        freq      = p["SINE_FREQ"]
        print(f"  Sinusoidal sweep: amp={amp:.2f} rad  freq={freq:.2f} Hz  ({sine_dur:.0f}s)")
        seg_start = time.monotonic()
        while time.monotonic() - seg_start < sine_dur:
            if shutdown_requested:
                break
            loop_start = time.monotonic()
            t_seg  = time.monotonic() - seg_start
            q_cmd  = amp * math.sin(2 * math.pi * freq * t_seg)

            send_mit(bus, mid, q_cmd, 0.0, p["KP"], p["KD"], 0.0)
            fb = read_feedback(bus, mid)

            t_now = time.monotonic() - t0
            if fb is not None:
                pos, vel, tau = fb
                log_rows.append({
                    "t":          round(t_now, 5),
                    "q_cmd":      round(q_cmd, 6),
                    "q_actual":   round(pos, 6),
                    "dq_actual":  round(vel, 6),
                    "tau_actual": round(tau, 6),
                })

            elapsed = time.monotonic() - loop_start
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    # ── Disable and save ─────────────────────────────────────────────────────
    print("\nDisabling motor...")
    disable_motor(bus, mid)
    bus.shutdown()

    if not log_rows:
        print("No data collected.")
        sys.exit(1)

    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["t", "q_cmd", "q_actual", "dq_actual", "tau_actual"])
        writer.writeheader()
        writer.writerows(log_rows)

    print(f"\nSaved {len(log_rows)} samples → {out_path}")
    print("Transfer to laptop and run: python deployment/actuator_ident/plot_comparison.py --csv <file>")

if __name__ == "__main__":
    main()
