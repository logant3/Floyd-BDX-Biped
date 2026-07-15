"""Interactive keyboard teleoperation for Floyd in Isaac Lab sim.

Controls:
    Up Arrow / Numpad 8    : forward
    Down Arrow / Numpad 2  : backward
    Left Arrow / Numpad 4  : strafe left
    Right Arrow / Numpad 6 : strafe right
    Z / Numpad 7           : rotate left (yaw)
    X / Numpad 9           : rotate right (yaw)
    L                      : stop (zero all commands)
    P                      : apply random push (tests recovery)
    ESC                    : quit
"""

import argparse
import sys

from isaaclab.app import AppLauncher

import cli_args  # isort: skip

parser = argparse.ArgumentParser(description="Keyboard teleoperation for Floyd.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments.")
parser.add_argument("--task", type=str, default="Floyd-Velocity-Flat-Play-v0")
parser.add_argument(
    "--agent", type=str, default="rsl_rl_cfg_entry_point",
)
parser.add_argument("--real-time", action="store_true", default=True)
cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import math
import os
import time

import carb
import carb.input
import gymnasium as gym
import torch
from rsl_rl.runners import OnPolicyRunner

from isaaclab.devices import Se2Keyboard
from isaaclab.devices.keyboard.se2_keyboard import Se2KeyboardCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.utils.assets import retrieve_file_path

from isaaclab_rl.rsl_rl import RslRlBaseRunnerCfg, RslRlVecEnvWrapper
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

import BDXR.tasks  # noqa: F401


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg, agent_cfg: RslRlBaseRunnerCfg):
    task_name = args_cli.task.split(":")[-1]
    train_task_name = task_name.replace("-Play", "")

    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    # Force command resampling to never auto-resample (we control it manually)
    env_cfg.commands.base_velocity.resampling_time_range = (1e9, 1e9)
    env_cfg.commands.base_velocity.rel_standing_envs = 0.0

    log_root_path = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))

    if args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    log_dir = os.path.dirname(resume_path)
    env_cfg.log_dir = log_dir

    # Create env
    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    # Load policy
    print(f"[INFO] Loading checkpoint: {resume_path}")
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(resume_path)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    try:
        policy_nn = runner.alg.policy
    except AttributeError:
        policy_nn = runner.alg.actor_critic

    device = env.unwrapped.device
    num_envs = env.unwrapped.num_envs

    # Keyboard device — must be created after sim starts
    teleop_cfg = Se2KeyboardCfg(
        v_x_sensitivity=0.4,
        v_y_sensitivity=0.35,
        omega_z_sensitivity=0.8,
        sim_device=str(device),
    )
    teleop = Se2Keyboard(teleop_cfg)
    teleop.reset()

    # Push key (P) — applies a random velocity impulse to test recovery
    push_requested = [False]
    _carb_input = carb.input.acquire_input_interface()

    import omni.appwindow
    _keyboard = omni.appwindow.get_default_app_window().get_keyboard()

    def _on_key_event(event, *args, **kwargs):
        if (event.type == carb.input.KeyboardEventType.KEY_PRESS
                and event.input == carb.input.KeyboardInput.P):
            push_requested[0] = True
        return True

    _keyboard_sub = _carb_input.subscribe_to_keyboard_events(_keyboard, _on_key_event)

    print("\n" + "=" * 50)
    print("  FLOYD KEYBOARD TELEOP")
    print("=" * 50)
    print("  Up / Down Arrow  : forward / backward")
    print("  Left / Right Arrow: strafe left / right")
    print("  Z / X            : rotate left / right")
    print("  L                : stop")
    print("  P                : random push (test recovery)")
    print("  ESC              : quit")
    print("=" * 50 + "\n")

    dt = env.unwrapped.step_dt
    obs = env.get_observations()

    while simulation_app.is_running():
        start_time = time.time()

        # advance() returns absolute command tensor (keys held = command active)
        cmd = teleop.advance()  # shape (3,) on device

        # Override velocity command in the environment
        try:
            vel_cmd_term = env.unwrapped.command_manager.get_term("base_velocity")
            vel_cmd_term.vel_command_b[:, :3] = cmd.unsqueeze(0).expand(num_envs, -1)
        except Exception:
            pass

        # Apply random push if P was pressed
        if push_requested[0]:
            push_requested[0] = False
            robot = env.unwrapped.scene["robot"]
            angle = torch.rand(1).item() * 2 * math.pi
            # Training push: push_by_setting_velocity with x/y in (-0.5, 0.5) m/s absolute.
            # Match that range so the policy sees a disturbance it was trained on.
            magnitude = 0.5  # m/s — matches training push_robot velocity_range max
            lin_vel = robot.data.root_lin_vel_w.clone()
            # SET (not add) lateral velocity to match training behavior
            lin_vel[:, 0] = magnitude * math.cos(angle)
            lin_vel[:, 1] = magnitude * math.sin(angle)
            ang_vel = robot.data.root_ang_vel_w.clone()
            # write_root_velocity_to_sim expects [N, 6]: linear then angular
            root_vel_6 = torch.cat([lin_vel, ang_vel], dim=-1)
            robot.write_root_velocity_to_sim(root_vel_6)
            print("\n  [PUSH]", end="", flush=True)

        with torch.inference_mode():
            actions = policy(obs)
            obs, _, dones, _ = env.step(actions)
            policy_nn.reset(dones)

        print(
            f"\rCmd: vx={cmd[0]:+.2f}  vy={cmd[1]:+.2f}  wz={cmd[2]:+.2f}   ",
            end="",
            flush=True,
        )

        sleep_time = dt - (time.time() - start_time)
        if sleep_time > 0:
            time.sleep(sleep_time)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
