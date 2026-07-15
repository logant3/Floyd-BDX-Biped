"""
Floyd Power Analysis Script
----------------------------
Runs the trained policy and logs per-joint mechanical power each step.
At the end, prints a power budget summary so you know real motor load %.

Usage (from Floyd-IsaacLab/):
    python scripts/power_analysis.py --task BDXR-Floyd-v0-Play --num_envs 1 \
        --checkpoint <path_to_checkpoint.pt> --steps 2000

Motor rated power (used for load % calculation):
    RS03 (hip_roll, hip_pitch, knee): 380W each  -> 6 motors = 2280W
    RS02 (ankle):                     170W each  -> 2 motors =  340W
    Total rated:                                              2620W
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import sys
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Floyd power analysis during policy rollout.")
parser.add_argument("--task",        type=str,   default=None)
parser.add_argument("--num_envs",    type=int,   default=1)
parser.add_argument("--checkpoint",  type=str,   default=None)
parser.add_argument("--steps",       type=int,   default=2000,  help="How many sim steps to log")
parser.add_argument("--agent",       type=str,   default="rsl_rl_cfg_entry_point")
parser.add_argument("--seed",        type=int,   default=None)
parser.add_argument("--real-time",   action="store_true", default=False)

# Append AppLauncher args so Isaac Sim initialises correctly
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Everything below runs after Isaac Sim is up."""

import time
import csv
import torch
import gymnasium as gym

from rsl_rl.runners import OnPolicyRunner, DistillationRunner

from isaaclab.envs import ManagerBasedRLEnvCfg, DirectRLEnvCfg, DirectMARLEnvCfg, multi_agent_to_single_agent
from isaaclab.utils.assets import retrieve_file_path
from isaaclab_rl.rsl_rl import RslRlBaseRunnerCfg, RslRlVecEnvWrapper
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import BDXR.tasks  # noqa: F401 — registers Floyd tasks

# ── Motor rated power ──────────────────────────────────────────────────────────
# Edit these if your joint names differ
RS03_JOINTS = [
    "left_hip_roll", "right_hip_roll",
    "left_hip_pitch", "right_hip_pitch",
    "left_knee",      "right_knee",
]
RS02_JOINTS = ["left_ankle", "right_ankle"]

RS03_RATED_W  = 380.0
RS02_RATED_W  = 170.0
MOTOR_EFF     = 0.80          # assumed electrical efficiency
BATTERY_V     = 40.0          # nominal volts

# Torque constants from RS03/RS02 datasheet — used to compute implied phase
# current directly from sim torque: I_phase (Arms) = torque (Nm) / Kt
RS03_KT = 2.36   # N·m/Arms  (RS03 datasheet)
RS02_KT = 1.22   # N·m/Arms  (RS02 datasheet)

TOTAL_RATED_MECHANICAL_W = len(RS03_JOINTS) * RS03_RATED_W + len(RS02_JOINTS) * RS02_RATED_W
# = 6*380 + 2*170 = 2620W
# ──────────────────────────────────────────────────────────────────────────────


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg,
         agent_cfg: RslRlBaseRunnerCfg):

    # ── Setup ──────────────────────────────────────────────────────────────────
    env_cfg.scene.num_envs = args_cli.num_envs
    if args_cli.seed is not None:
        env_cfg.seed = args_cli.seed
    env_cfg.sim.device = args_cli.device if args_cli.device else env_cfg.sim.device

    log_root_path = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))
    if args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    print(f"[power_analysis] Checkpoint: {resume_path}")

    env = gym.make(args_cli.task, cfg=env_cfg)
    if hasattr(env.unwrapped, "__class__") and "DirectMARL" in type(env.unwrapped).__name__:
        env = multi_agent_to_single_agent(env)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    # Load policy
    if agent_cfg.class_name == "OnPolicyRunner":
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    elif agent_cfg.class_name == "DistillationRunner":
        runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    else:
        raise ValueError(f"Unknown runner class: {agent_cfg.class_name}")
    runner.load(resume_path)
    policy    = runner.get_inference_policy(device=env.unwrapped.device)
    policy_nn = getattr(runner.alg, "policy", getattr(runner.alg, "actor_critic", None))

    # ── Find joint indices for RS03 / RS02 ────────────────────────────────────
    robot       = env.unwrapped.scene["robot"]
    joint_names = robot.data.joint_names   # list of strings, ordered by Isaac Lab

    def indices_for(names):
        idx = []
        for n in names:
            if n in joint_names:
                idx.append(joint_names.index(n))
            else:
                print(f"[WARN] joint '{n}' not found in robot. Available: {joint_names}")
        return idx

    rs03_idx = indices_for(RS03_JOINTS)
    rs02_idx = indices_for(RS02_JOINTS)
    all_idx  = rs03_idx + rs02_idx

    print(f"[power_analysis] Tracking {len(all_idx)} joints: {[joint_names[i] for i in all_idx]}")
    print(f"[power_analysis] Total rated mechanical power: {TOTAL_RATED_MECHANICAL_W:.0f}W")
    print(f"[power_analysis] Running for {args_cli.steps} steps...\n")

    # ── Rollout + logging ──────────────────────────────────────────────────────
    obs      = env.get_observations()
    dt       = env.unwrapped.step_dt

    # Storage for per-step scalars (env 0 only)
    mech_power_log   = []   # total mechanical watts across all joints
    elec_power_log   = []   # mechanical / efficiency
    current_log      = []   # electrical watts / battery voltage
    load_frac_log    = []   # mechanical / rated

    # Per-joint peak trackers
    peak_power_per_joint   = torch.zeros(len(all_idx))
    peak_torque_per_joint  = torch.zeros(len(all_idx))   # N·m
    peak_current_per_joint = torch.zeros(len(all_idx))   # Arms via Kt

    # Build per-joint Kt lookup (indexed same as all_idx)
    tracked_names_pre = [joint_names[i] for i in all_idx]
    kt_per_joint = torch.tensor([
        RS03_KT if n in RS03_JOINTS else RS02_KT
        for n in tracked_names_pre
    ])

    step = 0
    while simulation_app.is_running() and step < args_cli.steps:
        start = time.time()
        with torch.inference_mode():
            actions             = policy(obs)
            obs, _, dones, _    = env.step(actions)
            policy_nn.reset(dones)

        # Grab torque and velocity from Isaac Lab (env 0)
        # applied_torque: (num_envs, num_joints)
        # joint_vel:      (num_envs, num_joints)
        torques = robot.data.applied_torque[0]   # (num_joints,)
        vels    = robot.data.joint_vel[0]         # (num_joints,)

        # Per-joint mechanical power (can be negative during braking — use abs)
        per_joint_mech = torch.abs(torques * vels)   # (num_joints,)

        # Sum only our tracked joints
        tracked_mech   = per_joint_mech[all_idx]
        tracked_torque = torch.abs(torques[all_idx])          # N·m, per tracked joint
        tracked_kt_cur = tracked_torque.cpu() / kt_per_joint  # Arms via Kt, per joint
        total_mech     = tracked_mech.sum().item()

        # Update peak per joint
        peak_power_per_joint   = torch.max(peak_power_per_joint,   tracked_mech.cpu())
        peak_torque_per_joint  = torch.max(peak_torque_per_joint,  tracked_torque.cpu())
        peak_current_per_joint = torch.max(peak_current_per_joint, tracked_kt_cur)

        total_elec    = total_mech / MOTOR_EFF
        total_current = total_elec / BATTERY_V
        load_frac     = total_mech / TOTAL_RATED_MECHANICAL_W

        mech_power_log.append(total_mech)
        elec_power_log.append(total_elec)
        current_log.append(total_current)
        load_frac_log.append(load_frac)

        if step % 200 == 0:
            print(f"  step {step:4d} | mech {total_mech:6.1f}W | "
                  f"elec {total_elec:6.1f}W | "
                  f"current {total_current:5.2f}A | "
                  f"load {load_frac*100:5.1f}%")

        sleep_time = dt - (time.time() - start)
        if args_cli.real_time and sleep_time > 0:
            time.sleep(sleep_time)

        step += 1

    # ── Summary ───────────────────────────────────────────────────────────────
    if mech_power_log:
        mech_arr    = torch.tensor(mech_power_log)
        elec_arr    = torch.tensor(elec_power_log)
        curr_arr    = torch.tensor(current_log)
        load_arr    = torch.tensor(load_frac_log)

        print("\n" + "="*60)
        print("FLOYD POWER BUDGET SUMMARY")
        print("="*60)
        print(f"  Steps logged        : {step}")
        print(f"  Total rated power   : {TOTAL_RATED_MECHANICAL_W:.0f}W mechanical")
        print()
        print(f"  Mechanical power    — avg: {mech_arr.mean():.1f}W   peak: {mech_arr.max():.1f}W")
        print(f"  Electrical power    — avg: {elec_arr.mean():.1f}W   peak: {elec_arr.max():.1f}W")
        print(f"  Battery current     — avg: {curr_arr.mean():.2f}A  peak: {curr_arr.max():.2f}A")
        print(f"  Load fraction       — avg: {load_arr.mean()*100:.1f}%   peak: {load_arr.max()*100:.1f}%")
        print()
        print(f"  BMS limit (~22A)    — avg margin: {22 - curr_arr.mean():.2f}A  "
              f"peak margin: {22 - curr_arr.max():.2f}A")
        print()
        print("  Peak per joint (power | torque | implied phase current via Kt):")
        tracked_names = [joint_names[i] for i in all_idx]
        for name, pwr, trq, cur in zip(tracked_names,
                                        peak_power_per_joint.tolist(),
                                        peak_torque_per_joint.tolist(),
                                        peak_current_per_joint.tolist()):
            motor_type = "RS03" if name in RS03_JOINTS else "RS02"
            rated      = RS03_RATED_W if motor_type == "RS03" else RS02_RATED_W
            kt         = RS03_KT      if motor_type == "RS03" else RS02_KT
            rated_trq  = 60.0         if motor_type == "RS03" else 17.0
            print(f"    {name:<22s}  {pwr:6.1f}W ({pwr/rated*100:.0f}%)  |  "
                  f"{trq:5.1f} N·m ({trq/rated_trq*100:.0f}% of peak)  |  "
                  f"{cur:5.2f} Arms  [Kt={kt}]")
        print("="*60)

        # Save CSV next to checkpoint
        csv_path = os.path.join(os.path.dirname(resume_path), "power_log.csv")
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["step", "mech_W", "elec_W", "current_A", "load_frac"])
            for i, (m, e, c, l) in enumerate(zip(mech_power_log, elec_power_log,
                                                   current_log, load_frac_log)):
                writer.writerow([i, round(m, 3), round(e, 3), round(c, 4), round(l, 4)])
        print(f"\n  Full log saved to: {csv_path}")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
