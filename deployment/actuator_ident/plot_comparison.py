"""
plot_comparison.py — Sim vs Real actuator comparison plots
===========================================================
Produces a Skyentific-style layout:
  • Grid of individual step-response panels (each step gets its own plot)
  • Wide sine-sweep panel at the bottom
  • Each panel shows: command (gray dashed) / real (orange) / sim (green)

Segments are matched by their q_cmd value and aligned on relative time
within each segment — so timing offsets between the two CSVs don't matter.

Usage:
    python deployment/actuator_ident/plot_comparison.py \\
        --real right_knee_bench.csv \\
        --sim  RS03_sim_bench.csv

Only --real is required. If --sim is omitted, only real data is plotted.
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
# Style
# ──────────────────────────────────────────────────────────────────────────────

C_CMD  = "#888888"
C_REAL = "#d45f00"
C_SIM  = "#2f8c3f"

MOTOR_PARAMS = {
    "O2": {"kp": 16.581, "kd": 1.056, "effort_limit": 17.0, "label": "RS02 (ankle)"},
    "O3": {"kp": 78.957, "kd": 5.027, "effort_limit": 60.0, "label": "RS03 (hip/knee)"},
}


# ──────────────────────────────────────────────────────────────────────────────
# I/O
# ──────────────────────────────────────────────────────────────────────────────

def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            rows.append({k: float(v) for k, v in row.items()})
    return rows


def to_arrays(rows):
    return {k: np.array([r[k] for r in rows]) for k in rows[0]}


def infer_motor_type(path):
    lower = os.path.basename(path).lower()
    if "ankle" in lower or "rs02" in lower or "o2" in lower:
        return "O2"
    return "O3"


# ──────────────────────────────────────────────────────────────────────────────
# Segment detection
# Works on a single dataset. Returns step segments + sine segment.
# ──────────────────────────────────────────────────────────────────────────────

def detect_segments(data, min_duration=0.3):
    t     = data["t"]
    q_cmd = data["q_cmd"]
    dt    = float(np.median(np.diff(t)))
    min_len = max(5, int(min_duration / dt))

    # Find boundaries where q_cmd jumps
    boundaries = [0]
    for i in range(1, len(t)):
        if abs(q_cmd[i] - q_cmd[i - 1]) > 0.05:
            boundaries.append(i)
    boundaries.append(len(t))

    step_segs = []
    sine_seg  = None

    for k in range(len(boundaries) - 1):
        s, e = boundaries[k], boundaries[k + 1] - 1
        if (e - s) < min_len:
            continue
        chunk   = q_cmd[s:e + 1]
        cmd_val = round(float(np.median(chunk)), 3)
        # Detect sine: many direction changes in q_cmd itself
        direction_changes = int(np.sum(np.diff(np.sign(np.diff(chunk))) != 0))
        if direction_changes > 8:
            if sine_seg is None:  # take the first sine-like segment
                sine_seg = (s, e)
        else:
            step_segs.append((s, e, cmd_val))

    return step_segs, sine_seg


# ──────────────────────────────────────────────────────────────────────────────
# Segment matching
# Match real step segments to sim step segments by closest q_cmd value.
# Each real segment gets a (possibly None) sim counterpart.
# ──────────────────────────────────────────────────────────────────────────────

def match_segments(real_segs, sim_segs, tol=0.05):
    """
    Returns list of (real_seg, sim_seg_or_None) pairs.
    Sim seg is aligned on relative time — no absolute-time dependency.
    """
    pairs = []
    for r_seg in real_segs:
        _, _, r_cmd = r_seg
        best = None
        best_dist = tol
        for s_seg in sim_segs:
            _, _, s_cmd = s_seg
            dist = abs(r_cmd - s_cmd)
            if dist < best_dist:
                best_dist = dist
                best = s_seg
        pairs.append((r_seg, best))
    return pairs


def get_sim_aligned(sim_data, s_seg, t_real_rel):
    """
    Resample sim q_actual for a matched segment onto the real segment's
    relative time axis. Both are treated as starting at t=0.
    """
    s, e, _ = s_seg
    t_s  = sim_data["t"][s:e + 1]
    q_s  = sim_data["q_actual"][s:e + 1]
    t_s_rel = t_s - t_s[0]
    # Clamp t_real_rel to sim's duration
    t_query = np.clip(t_real_rel, 0.0, t_s_rel[-1])
    return np.interp(t_query, t_s_rel, q_s)


def get_sim_sine_aligned(sim_data, sine_seg, t_real_rel):
    s, e = sine_seg
    t_s     = sim_data["t"][s:e + 1]
    q_s     = sim_data["q_actual"][s:e + 1]
    q_cmd_s = sim_data["q_cmd"][s:e + 1]
    t_s_rel = t_s - t_s[0]
    t_query = np.clip(t_real_rel, 0.0, t_s_rel[-1])
    return np.interp(t_query, t_s_rel, q_s), np.interp(t_query, t_s_rel, q_cmd_s)


# ──────────────────────────────────────────────────────────────────────────────
# Plotting helpers
# ──────────────────────────────────────────────────────────────────────────────

def plot_step_panel(ax, t_rel, q_cmd, q_real, q_sim, cmd_label):
    ax.plot(t_rel, q_cmd,  color=C_CMD,  lw=1.0, ls="--", alpha=0.7)
    ax.plot(t_rel, q_real, color=C_REAL, lw=1.5)
    if q_sim is not None:
        ax.plot(t_rel, q_sim, color=C_SIM, lw=1.2, alpha=0.9)

    ax.set_title(f"cmd={cmd_label:+.2f} rad", fontsize=7, pad=2)
    ax.tick_params(labelsize=6)
    ax.grid(True, alpha=0.25, linewidth=0.5)
    ax.axhline(0, color="k", lw=0.4, alpha=0.3)

    all_vals = [q_cmd, q_real] + ([q_sim] if q_sim is not None else [])
    all_vals = np.concatenate(all_vals)
    span = max(np.ptp(all_vals), 0.02)
    mid  = (all_vals.min() + all_vals.max()) / 2
    ax.set_ylim(mid - span * 0.65, mid + span * 0.65)
    ax.set_xlabel("t (s)", fontsize=6)
    ax.set_ylabel("pos (rad)", fontsize=6)


def plot_sine_panel(ax, t_rel, q_cmd_real, q_real, q_cmd_sim, q_sim):
    ax.plot(t_rel, q_cmd_real, color=C_CMD,  lw=1.0, ls="--", alpha=0.7, label="Command")
    ax.plot(t_rel, q_real,     color=C_REAL, lw=1.5,           label="Real")
    if q_sim is not None:
        ax.plot(t_rel, q_sim, color=C_SIM, lw=1.2, alpha=0.9, label="Sim")
    ax.grid(True, alpha=0.25, linewidth=0.5)
    ax.axhline(0, color="k", lw=0.4, alpha=0.3)
    ax.set_xlabel("t (s)", fontsize=8)
    ax.set_ylabel("Position (rad)", fontsize=8)
    ax.set_title("Sinusoidal sweep", fontsize=9)
    ax.legend(loc="upper right", fontsize=8)


# ──────────────────────────────────────────────────────────────────────────────
# Main figure builder
# ──────────────────────────────────────────────────────────────────────────────

def build_figure(real_data, sim_data, mtype, real_path, sim_path):
    p = MOTOR_PARAMS[mtype]

    # ── Detect segments in each dataset independently ─────────────────────────
    real_steps, real_sine = detect_segments(real_data)
    sim_steps,  sim_sine  = (detect_segments(sim_data) if sim_data else ([], None))

    # ── Match step segments by command value ──────────────────────────────────
    pairs = match_segments(real_steps, sim_steps)

    # Filter out very short or duplicate segments (keep first occurrence of each cmd)
    seen_cmds = set()
    filtered_pairs = []
    for (r_seg, s_seg) in pairs:
        _, _, cmd = r_seg
        cmd_key = round(cmd, 1)
        if cmd_key not in seen_cmds:
            seen_cmds.add(cmd_key)
            filtered_pairs.append((r_seg, s_seg))

    n_steps  = len(filtered_pairs)
    N_COLS   = min(5, max(3, n_steps))
    n_rows   = math.ceil(n_steps / N_COLS)
    has_sine = real_sine is not None

    total_rows = n_rows + (1 if has_sine else 0)
    fig = plt.figure(figsize=(N_COLS * 2.8, total_rows * 2.6 + 1.0))

    motor_label = f"{p['label']}  —  kp={p['kp']}  kd={p['kd']}  effort_limit=±{p['effort_limit']} Nm"
    src_label   = f"Real: {os.path.basename(real_path)}"
    if sim_path:
        src_label += f"  |  Sim: {os.path.basename(sim_path)}"
    fig.suptitle(f"Actuator Identification — {motor_label}\n{src_label}",
                 fontsize=9, fontweight="bold", y=0.99)

    gs = gridspec.GridSpec(
        total_rows, N_COLS, figure=fig,
        hspace=0.55, wspace=0.4,
        top=0.93, bottom=0.08,
    )

    # ── Step panels ───────────────────────────────────────────────────────────
    for idx, (r_seg, s_seg) in enumerate(filtered_pairs):
        row = idx // N_COLS
        col = idx %  N_COLS
        ax  = fig.add_subplot(gs[row, col])

        s_r, e_r, cmd = r_seg
        t_r   = real_data["t"][s_r:e_r + 1]
        t_rel = t_r - t_r[0]
        q_cmd = real_data["q_cmd"][s_r:e_r + 1]
        q_r   = real_data["q_actual"][s_r:e_r + 1]

        q_s = None
        if s_seg is not None and sim_data is not None:
            q_s = get_sim_aligned(sim_data, s_seg, t_rel)

        plot_step_panel(ax, t_rel, q_cmd, q_r, q_s, cmd)

    # ── Sine panel ────────────────────────────────────────────────────────────
    if has_sine:
        s_r, e_r = real_sine
        t_r      = real_data["t"][s_r:e_r + 1]
        t_rel    = t_r - t_r[0]
        q_cmd_r  = real_data["q_cmd"][s_r:e_r + 1]
        q_r      = real_data["q_actual"][s_r:e_r + 1]

        q_s      = None
        q_cmd_s  = q_cmd_r  # fallback
        if sim_sine is not None and sim_data is not None:
            q_s, q_cmd_s = get_sim_sine_aligned(sim_data, sim_sine, t_rel)

        ax_sine = fig.add_subplot(gs[n_rows, :])
        plot_sine_panel(ax_sine, t_rel, q_cmd_r, q_r, q_cmd_s, q_s)

    # ── Legend ────────────────────────────────────────────────────────────────
    handles = [
        Line2D([0], [0], color=C_CMD,  lw=1.2, ls="--", label="Command"),
        Line2D([0], [0], color=C_REAL, lw=1.5,           label="Real motor"),
    ]
    if sim_data is not None:
        handles.append(Line2D([0], [0], color=C_SIM, lw=1.2, label="Sim model"))
    fig.legend(handles=handles, loc="lower center", ncol=len(handles),
               fontsize=8, bbox_to_anchor=(0.5, 0.005))

    return fig


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--real", required=True,
                        help="CSV from collect_real.py (real motor data)")
    parser.add_argument("--sim", default=None,
                        help="CSV from sim_bench.py (ODE model)")
    parser.add_argument("--type", choices=["O2", "O3"], default=None,
                        help="Motor type (auto-detected from filename if omitted)")
    args = parser.parse_args()

    mtype = args.type or infer_motor_type(args.real)
    print(f"Motor type: {MOTOR_PARAMS[mtype]['label']}")

    real_data = to_arrays(load_csv(args.real))
    print(f"Real: {len(real_data['t'])} samples  ({real_data['t'][-1]:.1f}s)")

    sim_data = None
    if args.sim:
        sim_data = to_arrays(load_csv(args.sim))
        print(f"Sim:  {len(sim_data['t'])} samples  ({sim_data['t'][-1]:.1f}s)")

    fig = build_figure(real_data, sim_data, mtype, args.real, args.sim)

    out_png = args.real.replace(".csv", "_comparison.png")
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"Saved → {out_png}")
    plt.show()


if __name__ == "__main__":
    main()
