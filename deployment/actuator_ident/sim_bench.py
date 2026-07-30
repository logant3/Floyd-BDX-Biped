"""
sim_bench.py — Actuator model bench simulation (no IsaacLab/GPU required)
==========================================================================
Numerically integrates the exact ODE that IsaacLab's DelayedPDActuatorCfg
uses for a single joint with no load:

    (armature) * q_ddot = clip(kp*(q_des - q) + kd*(0 - dq),
                                -effort_limit, +effort_limit)

Runs the same step/sine sequence as collect_real.py and saves a CSV
in the same column format so plot_comparison.py can overlay both.

No GPU, no Isaac Sim, no conda required — just numpy + scipy:
    pip install numpy scipy

Usage:
    python deployment/actuator_ident/sim_bench.py --type O3
    python deployment/actuator_ident/sim_bench.py --type O2
"""

import argparse
import csv
import math
import os

import numpy as np
from scipy.integrate import solve_ivp

# ──────────────────────────────────────────────────────────────────────────────
# Parameters — must match BDXR/robots/floyd.py
# ──────────────────────────────────────────────────────────────────────────────

MOTOR_PARAMS = {
    "O2": {  # RS02 — ankles
        "kp":            16.581,
        "kd":            1.056,
        "effort_limit":  17.0,
        "armature":      0.0042,   # kg·m² reflected rotor inertia
        "label":         "RS02 (ankle)",
        "small_step":    0.15,
        "large_step":    0.30,
        "sine_amp":      0.25,
        "sine_freq":     0.3,
    },
    "O3": {  # RS03 — hips, knees
        "kp":            78.957,
        "kd":            5.027,
        "effort_limit":  60.0,
        "armature":      0.02,     # kg·m² reflected rotor inertia
        "label":         "RS03 (hip/knee)",
        "small_step":    0.20,
        "large_step":    0.45,
        "sine_amp":      0.40,
        "sine_freq":     0.3,
    },
}

LOG_DT    = 1.0 / 200.0   # 200 Hz output sample rate (matches collect_real.py)
HOLD_TIME = 2.0            # seconds per step command
RAMP_TIME = 1.0            # seconds initial ramp to zero (logged but uninteresting)


def build_step_sequence(p):
    """Same sequence as collect_real.py."""
    A    = p["small_step"]
    B    = p["large_step"]
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


def simulate_segment(q0, dq0, q_cmd, duration, p):
    """
    Integrate the PD actuator ODE over [0, duration] with fixed q_cmd.
    Returns arrays (t_out, q_out, dq_out, tau_out) sampled at LOG_DT.
    """
    kp  = p["kp"]
    kd  = p["kd"]
    lim = p["effort_limit"]
    I   = p["armature"]   # bench test: no load, dynamics dominated by rotor inertia

    def ode(t, state):
        q, dq = state
        tau   = float(np.clip(kp * (q_cmd - q) + kd * (0.0 - dq), -lim, lim))
        return [dq, tau / I]

    t_eval = np.arange(0.0, duration, LOG_DT)
    sol    = solve_ivp(
        ode,
        [0.0, duration],
        [q0, dq0],
        method="RK45",
        t_eval=t_eval,
        max_step=LOG_DT / 4,
        rtol=1e-8,
        atol=1e-10,
    )

    q_out  = sol.y[0]
    dq_out = sol.y[1]

    # Compute applied torque at each sample point
    tau_out = np.clip(kp * (q_cmd - q_out) + kd * (0.0 - dq_out), -lim, lim)

    return sol.t, q_out, dq_out, tau_out


def simulate_sine_segment(q0, dq0, p):
    """Simulate the sinusoidal sweep (q_cmd changes each timestep)."""
    amp    = p["sine_amp"]
    freq   = p["sine_freq"]
    kp     = p["kp"]
    kd     = p["kd"]
    lim    = p["effort_limit"]
    I      = p["armature"]
    cycles = 4
    dur    = cycles / freq

    # Euler integration at fine dt (sine changes continuously so can't use fixed q_cmd)
    fine_dt = LOG_DT / 10
    t_fine  = np.arange(0.0, dur, fine_dt)
    q, dq   = q0, dq0
    t_log, q_log, dq_log, tau_log, cmd_log = [], [], [], [], []
    next_log = 0.0

    for t in t_fine:
        q_cmd = amp * math.sin(2 * math.pi * freq * t)
        tau   = float(np.clip(kp * (q_cmd - q) + kd * (0.0 - dq), -lim, lim))
        q_ddot = tau / I
        q  += dq  * fine_dt
        dq += q_ddot * fine_dt

        if t >= next_log - fine_dt / 2:
            t_log.append(t)
            q_log.append(q)
            dq_log.append(dq)
            tau_log.append(tau)
            cmd_log.append(q_cmd)
            next_log += LOG_DT

    return (np.array(t_log), np.array(q_log),
            np.array(dq_log), np.array(tau_log), np.array(cmd_log))


def main():
    parser = argparse.ArgumentParser(description="Actuator bench simulation (ODE)")
    parser.add_argument("--type",   choices=["O2", "O3"], default="O3",
                        help="Motor type: O2=RS02(ankle)  O3=RS03(hip/knee)")
    parser.add_argument("--output", type=str, default=None,
                        help="Output CSV (default: RS02_sim_bench.csv or RS03_sim_bench.csv)")
    args = parser.parse_args()

    mtype    = args.type
    p        = MOTOR_PARAMS[mtype]
    out_path = args.output or f"{'RS02' if mtype == 'O2' else 'RS03'}_sim_bench.csv"

    print(f"\n=== Actuator Bench Simulation ===")
    print(f"  Motor:          {p['label']}")
    print(f"  kp={p['kp']}  kd={p['kd']}  effort_limit=±{p['effort_limit']} Nm")
    print(f"  armature={p['armature']} kg·m²  (bench: no load)")
    print(f"  Output:         {out_path}\n")

    log_rows = []
    t_offset = 0.0
    q, dq    = 0.0, 0.0

    # ── Ramp (start at 0, target 0 — just for timing alignment) ──────────────
    print("  Ramp to zero...")
    t_seg, q_seg, dq_seg, tau_seg = simulate_segment(0.0, 0.0, 0.0, RAMP_TIME, p)
    for i in range(len(t_seg)):
        log_rows.append({
            "t":          round(float(t_seg[i]) + t_offset, 5),
            "q_cmd":      0.0,
            "q_actual":   round(float(q_seg[i]),   6),
            "dq_actual":  round(float(dq_seg[i]),  6),
            "tau_actual": round(float(tau_seg[i]), 6),
        })
    q, dq    = float(q_seg[-1]), float(dq_seg[-1])
    t_offset += RAMP_TIME

    # ── Step sequence ─────────────────────────────────────────────────────────
    for (q_cmd_val, duration) in build_step_sequence(p):
        print(f"  Step  q_cmd={q_cmd_val:+.3f} rad  ({duration:.1f}s)")
        t_seg, q_seg, dq_seg, tau_seg = simulate_segment(q, dq, q_cmd_val, duration, p)
        for i in range(len(t_seg)):
            log_rows.append({
                "t":          round(float(t_seg[i]) + t_offset, 5),
                "q_cmd":      round(q_cmd_val, 6),
                "q_actual":   round(float(q_seg[i]),   6),
                "dq_actual":  round(float(dq_seg[i]),  6),
                "tau_actual": round(float(tau_seg[i]), 6),
            })
        q, dq    = float(q_seg[-1]), float(dq_seg[-1])
        t_offset += duration

    # ── Sinusoidal sweep ──────────────────────────────────────────────────────
    print(f"  Sine  amp={p['sine_amp']} rad  freq={p['sine_freq']} Hz")
    t_seg, q_seg, dq_seg, tau_seg, cmd_seg = simulate_sine_segment(q, dq, p)
    for i in range(len(t_seg)):
        log_rows.append({
            "t":          round(float(t_seg[i]) + t_offset, 5),
            "q_cmd":      round(float(cmd_seg[i]), 6),
            "q_actual":   round(float(q_seg[i]),   6),
            "dq_actual":  round(float(dq_seg[i]),  6),
            "tau_actual": round(float(tau_seg[i]), 6),
        })

    # ── Save ─────────────────────────────────────────────────────────────────
    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(
            f, fieldnames=["t", "q_cmd", "q_actual", "dq_actual", "tau_actual"])
        writer.writeheader()
        writer.writerows(log_rows)

    print(f"\nSaved {len(log_rows)} samples → {out_path}")
    print(f"\nPlot: python deployment/actuator_ident/plot_comparison.py "
          f"--real <bench_csv> --sim {out_path}")


if __name__ == "__main__":
    main()
