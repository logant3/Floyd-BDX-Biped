"""
BDX-style walking gait generation for Floyd using placo IK.

Run on Linux inside the floyd_il venv:
    cd Floyd-BDX-Biped
    python BDXR/tasks/bdxr_locomotion/il_work/generate_gait.py

Outputs polynomial_coefficients.pkl matching the format expected by
reference_motion_floyd.py (24 dims, power-series polynomials, numpy 1.26.4).

pkl format:
  keys: "vx_vy_vtheta"  (e.g. "0.3_0.0_0.0")
  entry["period"]       : float, gait cycle duration (s)
  entry["coefficients"] : dict  dim_0 ... dim_23, each a list of n_c floats
  entry["fps"]          : int
  entry["nb_steps_in_period"] : int

24 state dims:
  0-7   jpos  [L_hip_roll, L_hip_pitch, L_knee, L_ankle,
               R_hip_roll, R_hip_pitch, R_knee, R_ankle]
  8-15  jvel  (same order)
  16-17 foot_contacts [left, right]  (0.0 = air, 1.0 = contact)
  18-20 base_lin_vel  [x, y, z]
  21-23 base_ang_vel  [x, y, z]
"""

import numpy as np
import pickle
import placo
from placo import FrameTask, KinematicsSolver, RobotWrapper

# ── Config ─────────────────────────────────────────────────────────────────────
URDF_PATH     = "BDXR/robots/urdf/Floyd_URDF.urdf"
DT            = 0.01          # s  (100 Hz)
CYCLE_TIME    = 0.5           # s  gait period
N_CYCLES      = 6             # cycles to record (skip first 2 for settling)
STEP_HEIGHT   = 0.055         # m  foot lift
BASE_HEIGHT   = 0.195         # m  pelvis height above ground
CONTACT_FORCE_THR = 0.01      # threshold to call foot "in contact"
POLY_DEGREE   = 15            # polynomial degree (fit to n_c=16 coefficients)
FPS           = int(1 / DT)

# Velocity sweep — (vx, vy, vtheta) tuples
# Start simple: forward walking at a few speeds
VELOCITY_CLIPS = [
    (0.0,  0.0,  0.0),   # stand still
    (0.15, 0.0,  0.0),   # slow forward
    (0.3,  0.0,  0.0),   # medium forward
    (0.5,  0.0,  0.0),   # fast forward
    (0.0,  0.2,  0.0),   # lateral
    (0.0, -0.2,  0.0),   # lateral other
]

# URDF joint name → placo/poly order index
# Poly order: [L_roll, L_pitch, L_knee, L_ankle, R_roll, R_pitch, R_knee, R_ankle]
URDF_JOINTS = {
    "left_roll":   "LeftHipRoll",
    "left_pitch":  "LeftHipPitch",
    "left_knee":   "LeftKneePitch",
    "left_ankle":  "LeftAnklePitch",
    "right_roll":  "RightHipRoll",
    "right_pitch": "RightHipPitch",
    "right_knee":  "RightKneePitch",
    "right_ankle": "RightAnklePitch",
}
JOINT_ORDER = list(URDF_JOINTS.keys())  # canonical poly order


def smooth_step(t):
    """Smooth step from 0→1, S-curve."""
    return t * t * (3 - 2 * t)


def swing_trajectory(phase, start_pos, end_pos, step_height):
    """
    phase: 0→1 during swing
    Returns 3D foot position.
    """
    alpha = smooth_step(phase)
    xy = (1 - alpha) * start_pos[:2] + alpha * end_pos[:2]
    z  = step_height * np.sin(np.pi * phase)
    return np.array([xy[0], xy[1], z])


def generate_clip(robot, vx, vy, vtheta, n_cycles, cycle_time, dt):
    """
    Generate one gait cycle for a given velocity command.
    Returns arrays: jpos (T,8), jvel (T,8), foot_contact (T,2),
                    base_lin_vel (T,3), base_ang_vel (T,3)
    """
    solver = KinematicsSolver(robot)
    solver.enable_velocity_limits(True)
    solver.dt = dt

    T_eye = np.eye(4)
    lf_task   = solver.add_frame_task("FootBaseLeft",  T_eye)
    rf_task   = solver.add_frame_task("FootBaseRight", T_eye)
    base_task = solver.add_frame_task("base_link",     T_eye)

    # position only (orientation weight = 0), soft constraint
    lf_task.configure("lf",   "soft", 2000.0, 0.0)
    rf_task.configure("rf",   "soft", 2000.0, 0.0)
    base_task.configure("base", "soft", 100.0, 50.0)

    joints_task = solver.add_joints_task()
    joints_task.configure("joints", "soft", 0.5)
    joints_task.set_joints({"LeftHipRoll": 0.0, "RightHipRoll": 0.0})

    # Reset to neutral
    for name in URDF_JOINTS.values():
        robot.set_joint(name, 0.0)
    robot.update_kinematics()

    lf0 = robot.get_T_world_fbase("FootBaseLeft")[:3, 3].copy();  lf0[2] = 0.0
    rf0 = robot.get_T_world_fbase("FootBaseRight")[:3, 3].copy(); rf0[2] = 0.0
    base0 = np.array([0.0, 0.0, BASE_HEIGHT])

    n_steps = int(n_cycles * cycle_time / dt)
    jpos_all    = np.zeros((n_steps, 8))
    jvel_all    = np.zeros((n_steps, 8))
    contact_all = np.zeros((n_steps, 2))
    linvel_all  = np.zeros((n_steps, 3))
    angvel_all  = np.zeros((n_steps, 3))

    lf_pos = lf0.copy()
    rf_pos = rf0.copy()
    base_pos = base0.copy()
    step_len = vx * cycle_time * 0.5   # distance per step
    step_lat = vy * cycle_time * 0.5

    prev_lf = lf_pos.copy()
    prev_rf = rf_pos.copy()
    prev_base = base_pos.copy()

    for i in range(n_steps):
        t     = i * dt
        phase = (t % cycle_time) / cycle_time  # [0, 1)

        if phase < 0.5:
            # Left swinging
            t_sw = phase / 0.5
            target_lf = lf_pos.copy()
            target_lf[0] += step_len
            target_lf[1] += step_lat

            lf_cmd = swing_trajectory(t_sw, lf_pos, target_lf, STEP_HEIGHT)
            rf_cmd = rf_pos.copy(); rf_cmd[2] = 0.0
            lf_contact = 0.0; rf_contact = 1.0

            # Commit step at end of swing
            if t_sw > 0.95:
                lf_pos = target_lf.copy(); lf_pos[2] = 0.0
        else:
            # Right swinging
            t_sw = (phase - 0.5) / 0.5
            target_rf = rf_pos.copy()
            target_rf[0] += step_len
            target_rf[1] += step_lat

            rf_cmd = swing_trajectory(t_sw, rf_pos, target_rf, STEP_HEIGHT)
            lf_cmd = lf_pos.copy(); lf_cmd[2] = 0.0
            lf_contact = 1.0; rf_contact = 0.0

            if t_sw > 0.95:
                rf_pos = target_rf.copy(); rf_pos[2] = 0.0

        base_mid_x = (lf_cmd[0] + rf_cmd[0]) / 2.0
        base_mid_y = (lf_cmd[1] + rf_cmd[1]) / 2.0

        def make_T(pos):
            T = np.eye(4)
            T[:3, 3] = pos
            return T

        lf_task.T_world_frame  = make_T(lf_cmd)
        rf_task.T_world_frame  = make_T(rf_cmd)
        base_task.T_world_frame = make_T(np.array([base_mid_x, base_mid_y, BASE_HEIGHT]))

        solver.solve(True)
        robot.update_kinematics()

        for j, key in enumerate(JOINT_ORDER):
            urdf_name = URDF_JOINTS[key]
            jpos_all[i, j] = robot.get_joint(urdf_name)

        # Finite-difference velocity
        if i > 0:
            jvel_all[i] = (jpos_all[i] - jpos_all[i-1]) / dt

        contact_all[i] = [lf_contact, rf_contact]

        cur_base = np.array([base_mid_x, base_mid_y, BASE_HEIGHT])
        if i > 0:
            linvel_all[i] = (cur_base - prev_base) / dt
        prev_base = cur_base.copy()

        # Angular velocity (approximate yaw rate)
        angvel_all[i, 2] = vtheta

    return jpos_all, jvel_all, contact_all, linvel_all, angvel_all


def fit_poly(traj_24, cycle_samples, degree):
    """
    Fit polynomial to averaged gait cycle.
    traj_24: (T, 24) full trajectory
    Returns dict of dim_0..dim_23, each a list of (degree+1) coefficients.
    """
    t_norm = np.linspace(0.0, 1.0, cycle_samples)
    coeffs_dict = {}
    for d in range(24):
        # Use power-series basis (match Kayden's format: c[0]*t^0 + c[1]*t^1 + ...)
        # np.polyfit returns highest power first → reverse
        c = np.polyfit(t_norm, traj_24[:, d], degree)[::-1]
        coeffs_dict[f"dim_{d}"] = c.tolist()
    return coeffs_dict


# ── Main ───────────────────────────────────────────────────────────────────────
print("Loading URDF...")
# Strip mesh references so placo doesn't try to load Windows paths
import re, tempfile, os
with open(URDF_PATH, "r", errors="replace") as f:
    urdf_str = f.read()
# Remove all <mesh .../> tags
urdf_str = re.sub(r'<mesh[^/]*/>', '', urdf_str)
# Remove <visual> and <collision> blocks entirely
urdf_str = re.sub(r'<visual>.*?</visual>', '', urdf_str, flags=re.DOTALL)
urdf_str = re.sub(r'<collision>.*?</collision>', '', urdf_str, flags=re.DOTALL)
_tmp = tempfile.NamedTemporaryFile(suffix=".urdf", delete=False, mode="w")
_tmp.write(urdf_str)
_tmp.close()
robot = RobotWrapper(_tmp.name, placo.Flags.ignore_collisions)
os.unlink(_tmp.name)

output = {}
cycle_samples = int(CYCLE_TIME / DT)

for (vx, vy, vtheta) in VELOCITY_CLIPS:
    key = f"{vx}_{vy}_{vtheta}"
    print(f"\nGenerating clip: vx={vx} vy={vy} vtheta={vtheta}")

    jpos, jvel, contact, linvel, angvel = generate_clip(
        robot, vx, vy, vtheta, N_CYCLES, CYCLE_TIME, DT
    )

    # Skip first 2 cycles (settling), average remaining
    skip = 2 * cycle_samples
    usable = jpos[skip:]
    n_use = len(usable) // cycle_samples * cycle_samples
    if n_use == 0:
        print(f"  WARNING: not enough samples, skipping")
        continue

    def avg_cycle(arr):
        n = len(arr) // cycle_samples * cycle_samples
        return arr[:n].reshape(-1, cycle_samples, arr.shape[-1]).mean(axis=0)

    jpos_avg    = avg_cycle(jpos[skip:skip+n_use])       # (cycle_samples, 8)
    jvel_avg    = avg_cycle(jvel[skip:skip+n_use])
    contact_avg = avg_cycle(contact[skip:skip+n_use])
    linvel_avg  = avg_cycle(linvel[skip:skip+n_use])
    angvel_avg  = avg_cycle(angvel[skip:skip+n_use])

    traj_24 = np.concatenate([
        jpos_avg, jvel_avg, contact_avg, linvel_avg, angvel_avg
    ], axis=1)  # (cycle_samples, 24)

    print(f"  jpos range: {jpos_avg.min():.3f} to {jpos_avg.max():.3f}")
    print(f"  contact pattern: {contact_avg[0].round(2)} → {contact_avg[cycle_samples//2].round(2)}")

    coeffs = fit_poly(traj_24, cycle_samples, POLY_DEGREE)

    output[key] = {
        "coefficients": coeffs,
        "period": CYCLE_TIME,
        "fps": FPS,
        "nb_steps_in_period": cycle_samples,
        "frame_offsets": [0],
        "start_offset": 0,
        "startend_double_support_ratio": 0.0,
    }

out_path = "BDXR/tasks/bdxr_locomotion/il_work/polynomial_coefficients.pkl"
with open(out_path, "wb") as f:
    pickle.dump(output, f)

print(f"\nDone. Saved {len(output)} clips to {out_path}")
print(f"numpy version: {np.__version__}  ← must be 1.26.4")
print("Clips:", list(output.keys()))
