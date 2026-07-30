"""
plot_comparison.py — Sim vs Real actuator comparison plots
===========================================================
Produces a Skyentific-style layout:
  • Grid of individual step-response panels (each step gets its own plot)
  • Wide sine-sweep panel at the bottom
  • Each panel shows: command (gray dashed) / real position (orange) / sim position (green)

Usage — after running both collect scripts:
    python deployment/actuator_ident/plot_comparison.py \\
        --real right_knee_bench.csv \\
        --sim  RS03_sim_bench.csv

Only --real is required. If --sim is omitted, the sim torque is computed
analytically from the PD formula (useful before running sim_bench.py).

The two CSVs must have columns: t, q_cmd, q_actual, dq_actual, tau_actual
"""

import argparse
import csv
import math
import os

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.lines import Line2D

# ──────────────────────────────────────────────────────────────────────────────
# Style — matches Skyentific video color scheme
# ──────────────────────────────────────────────────────────────────────────────

C_CMD  = "#888888"   # command (dashed gray)
C_REAL = "#d45f00"   # real motor (orange-red)
C_SIM  = "#2f8c3f"   # simulation (green)
C_SIM2 = "#2b7cb3"   # analytical sim fallback (blue)

MOTOR_PARAMS = {
    "O2": {"kp": 16.581, "kd": 1.056, "effort_limit": 17.0, "label": "RS02 (ankle)"},
    "O3": {"kp": 78.957, "kd": 5.027, "effort_limit": 60.0, "label": "RS03 (hip/knee)"},
}

HOLD_TIME = 2.0
SINE_FREQ = 0.3


# ──────────────────────────────────────────────────────────────────────────────
# I/O helpers
# ──────────────────────────────────────────────────────────────────────────────

def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            rows.append({k: float(v) for k, v in row.items()})
    return rows


def infer_motor_type(path):
    lower = os.path.basename(path).lower()
    if "ankle" in lower or "rs02" in lower or "o2" in lower:
        return "O2"
    return "O3"


def to_arrays(rows):
    return {k: np.array([r[k] for r in rows]) for k in rows[0]}


# ──────────────────────────────────────────────────────────────────────────────
# Segment detection
# Splits the time series into step segments + sine segment.
# We detect segment boundaries by large jumps in q_cmd.
# ──────────────────────────────────────────────────────────────────────────────

def detect_segments(t, q_cmd):
    """
    Returns list of (start_idx, end_idx, label, is_sine) for each segment.
    The sine segment is detected when q_cmd oscillates (changes direction many times).
    """
    segments = []
    n = len(t)
    if n == 0:
        return segments

    # Find transitions: indices where q_cmd changes by more than a threshold
    threshold = 0.05  # rad
    boundaries = [0]
    for i in range(1, n):
        if abs(q_cmd[i] - q_cmd[i - 1]) > threshold:
            boundaries.append(i)
    boundaries.append(n)

    # Group consecutive samples with same q_cmd into segments
    # But the sine section has continuously changing q_cmd, so we detect it
    # by checking if q_cmd has many zero crossings in a short window
    step_segs = []
    current_cmd = q_cmd[0]
    seg_start   = 0

    for i in range(1, n + 1):
        if i == n or abs(q_cmd[i] - current_cmd) > threshold:
            step_segs.append((seg_start, i - 1, current_cmd))
            seg_start   = i
            current_cmd = q_cmd[i] if i < n else 0.0

    # The sine segment: if a "segment" has more than ~10 direction changes,
    # it's the sine sweep rather than a held step
    result_steps = []
    result_sine  = None

    for (s, e, cmd) in step_segs:
        chunk = q_cmd[s:e + 1]
        direction_changes = np.sum(np.diff(np.sign(np.diff(chunk))) != 0)
        if direction_changes > 8:
            result_sine = (s, e)
        else:
            result_steps.append((s, e, round(float(cmd), 3)))

    return result_steps, result_sine


# ──────────────────────────────────────────────────────────────────────────────
# Plotting
# ──────────────────────────────────────────────────────────────────────────────

def plot_step_panel(ax, t, q_cmd, q_real, q_sim, label, effort_limit=None):
    """Single step-response panel."""
    t_rel = t - t[0]
    ax.plot(t_rel, q_cmd,  color=C_CMD,  lw=1.0, ls="--", alpha=0.7)
    ax.plot(t_rel, q_real, color=C_REAL, lw=1.5)
    if q_sim is not None:
        ax.plot(t_rel, q_sim, color=C_SIM, lw=1.2, alpha=0.85)

    ax.set_title(f"cmd={label:+.2f} rad", fontsize=7, pad=2)
    ax.tick_params(labelsize=6)
    ax.grid(True, alpha=0.25, linewidth=0.5)
    ax.axhline(0, color="k", lw=0.4, alpha=0.4)

    # Auto-scale y with a little padding
    all_vals = np.concatenate([q_cmd, q_real] + ([q_sim] if q_sim is not None else []))
    y_range  = max(np.ptp(all_vals), 0.02)
    y_mid    = np.mean([all_vals.min(), all_vals.max()])
    ax.set_ylim(y_mid - y_range * 0.7, y_mid + y_range * 0.7)
    ax.set_xlabel("t (s)", fontsize=6)
    ax.set_ylabel("pos (rad)", fontsize=6)


def plot_sine_panel(ax, t, q_cmd, q_real, q_sim, effort_limit=None):
    """Wide sine-sweep panel at the bottom."""
    t_rel = t - t[0]
    ax.plot(t_rel, q_cmd,  color=C_CMD,  lw=1.0, ls="--", alpha=0.7, label="Command")
    ax.plot(t_rel, q_real, color=C_REAL, lw=1.5, label="Real")
    if q_sim is not None:
        ax.plot(t_rel, q_sim, color=C_SIM, lw=1.2, alpha=0.85, label="Sim")
    ax.grid(True, alpha=0.25, linewidth=0.5)
    ax.axhline(0, color="k", lw=0.4, alpha=0.4)
    ax.set_xlabel("t (s)", fontsize=8)
    ax.set_ylabel("Position (rad)", fontsize=8)
    ax.set_title("Sinusoidal sweep", fontsize=9)
    ax.legend(loc="upper right", fontsize=8)


def build_figure(real, sim_data, mtype, real_path, sim_path):
    p    = MOTOR_PARAMS[mtype]
    name = os.path.basename(real_path)

    t_r     = real["t"]
    q_cmd_r = real["q_cmd"]
    q_r     = real["q_actual"]

    # Align sim to real by time if both provided; otherwise use real time
    if sim_data is not None:
        # Interpolate sim onto real's time axis for clean overlay
        t_s     = sim_data["t"]
        q_s_raw = sim_data["q_actual"]
        q_s     = np.interp(t_r, t_s, q_s_raw)
    else:
        # Analytical sim: compute from PD formula using real pos/vel
        kp, kd, lim = p["kp"], p["kd"], p["effort_limit"]
        dq_r   = real["dq_actual"]
        tau_an = np.clip(kp * (q_cmd_r - q_r) + kd * (0.0 - dq_r), -lim, lim)
        q_s    = None  # no sim position when using analytical torque only

    # ── Detect segments ───────────────────────────────────────────────────────
    step_segs, sine_seg = detect_segments(t_r, q_cmd_r)

    # Filter out the q_cmd=0 hold segments between steps — keep only non-zero steps
    # AND the zeros for context, but limit the zero panels to 1 each side
    # Actually: show ALL steps including zero holds so user sees full picture
    # but skip very short segments (< 0.3s)
    dt      = float(np.median(np.diff(t_r)))
    min_len = max(5, int(0.3 / dt))
    step_segs = [(s, e, cmd) for (s, e, cmd) in step_segs if (e - s) >= min_len]

    n_steps = len(step_segs)

    # ── Layout: rows of N_COLS, then sine at bottom ───────────────────────────
    N_COLS   = min(5, max(3, n_steps))
    n_rows   = math.ceil(n_steps / N_COLS)
    has_sine = sine_seg is not None

    total_rows = n_rows + (1 if has_sine else 0)
    fig = plt.figure(figsize=(N_COLS * 2.8, total_rows * 2.6 + 0.8))

    motor_label = f"{p['label']}  —  kp={p['kp']}  kd={p['kd']}  effort_limit=±{p['effort_limit']} Nm"
    src_label   = f"Real: {os.path.basename(real_path)}"
    if sim_path:
        src_label += f"  |  Sim: {os.path.basename(sim_path)}"
    fig.suptitle(f"Actuator Identification — {motor_label}\n{src_label}",
                 fontsize=9, fontweight="bold", y=0.99)

    gs = gridspec.GridSpec(
        total_rows, N_COLS, figure=fig,
        hspace=0.55, wspace=0.4,
        top=0.93, bottom=0.07
    )

    # ── Step panels ───────────────────────────────────────────────────────────
    for idx, (s, e, cmd) in enumerate(step_segs):
        row = idx // N_COLS
        col = idx %  N_COLS
        ax  = fig.add_subplot(gs[row, col])
        q_sim_seg = q_s[s:e + 1] if q_s is not None else None
        plot_step_panel(
            ax,
            t_r[s:e + 1],
            q_cmd_r[s:e + 1],
            q_r[s:e + 1],
            q_sim_seg,
            cmd,
            effort_limit=p["effort_limit"],
        )

    # ── Sine panel (full width at bottom) ─────────────────────────────────────
    if has_sine:
        s, e    = sine_seg
        ax_sine = fig.add_subplot(gs[n_rows, :])
        q_sim_sine = q_s[s:e + 1] if q_s is not None else None
        plot_sine_panel(
            ax_sine,
            t_r[s:e + 1],
            q_cmd_r[s:e + 1],
            q_r[s:e + 1],
            q_sim_sine,
            effort_limit=p["effort_limit"],
        )

    # ── Global legend ─────────────────────────────────────────────────────────
    legend_handles = [
        Line2D([0], [0], color=C_CMD,  lw=1.2, ls="--", label="Command"),
        Line2D([0], [0], color=C_REAL, lw=1.5,           label="Real motor"),
    ]
    if q_s is not None:
        legend_handles.append(Line2D([0], [0], color=C_SIM, lw=1.2, label="IsaacLab sim"))
    fig.legend(handles=legend_handles, loc="lower center",
               ncol=len(legend_handles), fontsize=8,
               bbox_to_anchor=(0.5, 0.005))

    return fig


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--real", required=True,
                        help="CSV from collect_real.py (real motor data)")
    parser.add_argument("--sim", default=None,
                        help="CSV from sim_bench.py (IsaacLab simulation). "
                             "If omitted, uses analytical PD formula for torque only.")
    parser.add_argument("--type", choices=["O2", "O3"], default=None,
                        help="Motor type (auto-detected from filename if omitted)")
    args = parser.parse_args()

    mtype = args.type or infer_motor_type(args.real)
    print(f"Motor type: {MOTOR_PARAMS[mtype]['label']}")

    real_rows = load_csv(args.real)
    real      = to_arrays(real_rows)
    print(f"Real data: {len(real_rows)} samples  ({real['t'][-1]:.1f}s)")

    sim_data = None
    if args.sim:
        sim_rows = load_csv(args.sim)
        sim_data = to_arrays(sim_rows)
        print(f"Sim data:  {len(sim_rows)} samples  ({sim_data['t'][-1]:.1f}s)")
    else:
        print("No sim CSV provided — using analytical PD torque (no sim position curve).")

    fig = build_figure(real, sim_data, mtype, args.real, args.sim)

    out_png = args.real.replace(".csv", "_comparison.png")
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"\nSaved → {out_png}")
    plt.show()


if __name__ == "__main__":
    main()
