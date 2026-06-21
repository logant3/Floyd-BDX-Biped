"""Imitation Learning reward terms for Floyd BDX-R.

These rewards compare the robot's current state to the polynomial reference motion
stored in the imitation command term.

Command layout (18 dims):
  0-1  : sin/cos phase
  2-9  : ref joint positions
  10-17: ref joint velocities
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


# pkl joint order: [LHipRoll, LHipPitch, LKnee, LAnkle, RHipRoll, RHipPitch, RKnee, RAnkle]
_PKL_JOINT_NAMES = [
    "LeftHipRoll", "LeftHipPitch", "LeftKneePitch", "LeftAnklePitch",
    "RightHipRoll", "RightHipPitch", "RightKneePitch", "RightAnklePitch",
]
_PKL_JOINT_IDS: torch.Tensor | None = None


def _get_joint_ids(asset) -> torch.Tensor:
    global _PKL_JOINT_IDS
    if _PKL_JOINT_IDS is None:
        name_to_idx = {n: i for i, n in enumerate(asset.data.joint_names)}
        _PKL_JOINT_IDS = torch.tensor(
            [name_to_idx[n] for n in _PKL_JOINT_NAMES], dtype=torch.long
        )
    return _PKL_JOINT_IDS


def imitate_joint_pos(
    env: ManagerBasedRLEnv,
    std: float = 0.25,
    command_name: str = "imitation",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reward for matching reference joint positions.

    Uses exp(-||q - q_ref||^2 / (2*std^2)).
    """
    asset = env.scene[asset_cfg.name]
    ids = _get_joint_ids(asset).to(asset.data.joint_pos.device)
    q = asset.data.joint_pos[:, ids]   # (B, 8) in pkl order

    cmd = env.command_manager.get_command(command_name)
    q_ref = cmd[:, 2:10]       # (B, 8)

    error = torch.sum((q - q_ref) ** 2, dim=-1)
    return torch.exp(-error / (2.0 * std ** 2))


def imitate_joint_vel(
    env: ManagerBasedRLEnv,
    std: float = 2.0,
    command_name: str = "imitation",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reward for matching reference joint velocities."""
    asset = env.scene[asset_cfg.name]
    ids = _get_joint_ids(asset).to(asset.data.joint_vel.device)
    qd = asset.data.joint_vel[:, ids]  # (B, 8) in pkl order

    cmd = env.command_manager.get_command(command_name)
    qd_ref = cmd[:, 10:18]    # (B, 8)

    error = torch.sum((qd - qd_ref) ** 2, dim=-1)
    return torch.exp(-error / (2.0 * std ** 2))


def imitate_foot_contact(
    env: ManagerBasedRLEnv,
    command_name: str = "imitation",
    threshold: float = 1.0,
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("contact_forces"),
) -> torch.Tensor:
    """Reward for matching reference foot contact pattern.

    +1 when actual contact matches reference, 0 otherwise, averaged over feet.
    """
    # Get the imitation command term to access foot_contacts
    cmd_term = env.command_manager._terms[command_name]
    ref_contacts = cmd_term.foot_contacts  # (B, 2), values in [0,1]
    ref_binary = (ref_contacts > 0.5).float()

    # Get actual contact forces — filtered to foot bodies only
    contact_sensor = env.scene.sensors[sensor_cfg.name]
    net_forces = contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, :]  # (B, 2, 3)
    actual_force_mag = torch.norm(net_forces, dim=-1)  # (B, 2)
    actual_binary = (actual_force_mag > threshold).float()

    # Average agreement across feet
    agreement = (actual_binary == ref_binary).float()
    return agreement.mean(dim=-1)
