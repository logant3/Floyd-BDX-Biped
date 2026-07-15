# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""This sub-module contains the reward functions that can be used for Spot's locomotion task.

The functions can be passed to the :class:`isaaclab.managers.RewardTermCfg` object to
specify the reward function and its parameters.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import ManagerTermBase, SceneEntityCfg
from isaaclab.sensors import ContactSensor
from isaaclab.utils.math import quat_apply_inverse

from .reference_motion import ReferenceMotionLibraryLegs, FLOYD_REF_INDICES

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.managers import RewardTermCfg


##
# Task Rewards
##


def both_feet_air_penalty(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg,
    threshold: float,
) -> torch.Tensor:
    """Penalize when both feet are simultaneously off the ground (jumping/hopping).

    This discourages the policy from discovering jumping as a local optimum
    before learning to walk with alternating foot contact.
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    net_contact_forces = contact_sensor.data.net_forces_w_history
    # True where foot is in contact (force above threshold)
    is_contact = torch.max(torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1)[0] > threshold
    # Penalize when neither foot is in contact
    both_in_air = ~is_contact[:, 0] & ~is_contact[:, 1]
    return both_in_air.float()


def feet_air_time_touchdown(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg,
    threshold_min: float = 0.3,
    threshold_max: float = 0.6,
) -> torch.Tensor:
    """Reward at touchdown based on completed air time (Skyentific-style).

    Fires once per foot landing. Negative for short hops (< threshold_min),
    positive and capped for good steps (threshold_min to threshold_max).
    Gated on xy velocity command so robot stands still when commanded to.
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    first_contact = contact_sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids]
    last_air_time = contact_sensor.data.last_air_time[:, sensor_cfg.body_ids]
    air_time = (last_air_time - threshold_min) * first_contact
    air_time = torch.clamp(air_time, min=0.0, max=threshold_max - threshold_min)
    reward = torch.sum(air_time, dim=1)
    reward *= torch.norm(env.command_manager.get_command("base_velocity")[:, :2], dim=1) > 0.1
    return reward


def feet_air_time_positive_biped(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg,
    threshold_min: float = 0.3,
    threshold_max: float = 0.6,
) -> torch.Tensor:
    """Continuously reward single-stance gait (exactly one foot on ground).

    While the robot is in single stance, reward accumulates proportional to
    how long it has been in that mode. Encourages sustained foot lift rather
    than quick taps. Gated on xy velocity command.
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    air_time = contact_sensor.data.current_air_time[:, sensor_cfg.body_ids]
    contact_time = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids]
    in_contact = contact_time > 0.0
    in_mode_time = torch.where(in_contact, contact_time, air_time)
    single_stance = torch.sum(in_contact.int(), dim=1) == 1
    reward = torch.min(torch.where(single_stance.unsqueeze(-1), in_mode_time, 0.0), dim=1)[0]
    reward = torch.clamp(reward, max=threshold_max)
    reward *= reward > threshold_min
    reward *= torch.norm(env.command_manager.get_command("base_velocity")[:, :2], dim=1) > 0.1
    return reward


def bipedal_air_time_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    sensor_cfg: SceneEntityCfg,
    mode_time: float,
    velocity_threshold: float,
) -> torch.Tensor:
    """Reward longer feet air and contact time for a bipedal robot."""
    # extract the used quantities (to enable type-hinting)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    asset: Articulation = env.scene[asset_cfg.name]
    if not contact_sensor.cfg.track_air_time:
        raise RuntimeError("Activate ContactSensor's track_air_time!")

    # Ensure you are tracking the correct body IDs for the two feet.
    # This will depend on your robot's URDF file.
    # For example, if your foot body IDs are 3 and 6:
    # sensor_cfg.body_ids = [3, 6]

    # compute the reward
    current_air_time = contact_sensor.data.current_air_time[:, sensor_cfg.body_ids]
    current_contact_time = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids]

    t_max = torch.max(current_air_time, current_contact_time)
    t_min = torch.clip(t_max, max=mode_time)
    stance_cmd_reward = torch.clip(current_contact_time - current_air_time, -mode_time, mode_time)

    # MODIFIED: Expand command to 2 dimensions for the two feet
    cmd = torch.norm(env.command_manager.get_command("base_velocity"), dim=1).unsqueeze(dim=1).expand(-1, 2)

    # MODIFIED: Expand body velocity to 2 dimensions for the two feet
    body_vel = torch.linalg.norm(asset.data.root_lin_vel_b[:, :2], dim=1).unsqueeze(dim=1).expand(-1, 2)

    reward = torch.where(
        torch.logical_or(cmd > 0.0, body_vel > velocity_threshold),
        torch.where(t_max < mode_time, t_min, 0),
        stance_cmd_reward,
    )
    return torch.sum(reward, dim=1)


def base_angular_velocity_reward(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, std: float) -> torch.Tensor:
    """Reward tracking of angular velocity commands (yaw) using abs exponential kernel."""
    # extract the used quantities (to enable type-hinting)
    asset: RigidObject = env.scene[asset_cfg.name]
    # compute the error
    target = env.command_manager.get_command("base_velocity")[:, 2]
    ang_vel_error = torch.linalg.norm((target - asset.data.root_ang_vel_b[:, 2]).unsqueeze(1), dim=1)
    return torch.exp(-ang_vel_error / std)


def base_linear_velocity_reward(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, std: float, ramp_at_vel: float = 1.0, ramp_rate: float = 0.5
) -> torch.Tensor:
    """Reward tracking of linear velocity commands (xy axes) using abs exponential kernel."""
    # extract the used quantities (to enable type-hinting)
    asset: RigidObject = env.scene[asset_cfg.name]
    # compute the error
    target = env.command_manager.get_command("base_velocity")[:, :2]
    lin_vel_error = torch.linalg.norm((target - asset.data.root_lin_vel_b[:, :2]), dim=1)
    # fixed 1.0 multiple for tracking below the ramp_at_vel value, then scale by the rate above
    vel_cmd_magnitude = torch.linalg.norm(target, dim=1)
    velocity_scaling_multiple = torch.clamp(1.0 + ramp_rate * (vel_cmd_magnitude - ramp_at_vel), min=1.0)
    return torch.exp(-lin_vel_error / std) * velocity_scaling_multiple


class GaitReward(ManagerTermBase):
    """Gait enforcing reward term for quadrupeds.

    This reward penalizes contact timing differences between selected foot pairs defined in :attr:`synced_feet_pair_names`
    to bias the policy towards a desired gait, i.e trotting, bounding, or pacing. Note that this reward is only for
    quadrupedal gaits with two pairs of synchronized feet.
    """

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
        """Initialize the term.

        Args:
            cfg: The configuration of the reward.
            env: The RL environment instance.
        """
        super().__init__(cfg, env)
        self.std: float = cfg.params["std"]
        self.max_err: float = cfg.params["max_err"]
        self.velocity_threshold: float = cfg.params["velocity_threshold"]
        self.contact_sensor: ContactSensor = env.scene.sensors[cfg.params["sensor_cfg"].name]
        self.asset: Articulation = env.scene[cfg.params["asset_cfg"].name]
        # match foot body names with corresponding foot body ids
        synced_feet_pair_names = cfg.params["synced_feet_pair_names"]
        if (
            len(synced_feet_pair_names) != 2
            or len(synced_feet_pair_names[0]) != 2
            or len(synced_feet_pair_names[1]) != 2
        ):
            raise ValueError("This reward only supports gaits with two pairs of synchronized feet, like trotting.")
        synced_feet_pair_0 = self.contact_sensor.find_bodies(synced_feet_pair_names[0])[0]
        synced_feet_pair_1 = self.contact_sensor.find_bodies(synced_feet_pair_names[1])[0]
        self.synced_feet_pairs = [synced_feet_pair_0, synced_feet_pair_1]

    def __call__(
        self,
        env: ManagerBasedRLEnv,
        std: float,
        max_err: float,
        velocity_threshold: float,
        synced_feet_pair_names,
        asset_cfg: SceneEntityCfg,
        sensor_cfg: SceneEntityCfg,
    ) -> torch.Tensor:
        """Compute the reward.

        This reward is defined as a multiplication between six terms where two of them enforce pair feet
        being in sync and the other four rewards if all the other remaining pairs are out of sync

        Args:
            env: The RL environment instance.
        Returns:
            The reward value.
        """
        # for synchronous feet, the contact (air) times of two feet should match
        sync_reward_0 = self._sync_reward_func(self.synced_feet_pairs[0][0], self.synced_feet_pairs[0][1])
        sync_reward_1 = self._sync_reward_func(self.synced_feet_pairs[1][0], self.synced_feet_pairs[1][1])
        sync_reward = sync_reward_0 * sync_reward_1
        # for asynchronous feet, the contact time of one foot should match the air time of the other one
        async_reward_0 = self._async_reward_func(self.synced_feet_pairs[0][0], self.synced_feet_pairs[1][0])
        async_reward_1 = self._async_reward_func(self.synced_feet_pairs[0][1], self.synced_feet_pairs[1][1])
        async_reward_2 = self._async_reward_func(self.synced_feet_pairs[0][0], self.synced_feet_pairs[1][1])
        async_reward_3 = self._async_reward_func(self.synced_feet_pairs[1][0], self.synced_feet_pairs[0][1])
        async_reward = async_reward_0 * async_reward_1 * async_reward_2 * async_reward_3
        # only enforce gait if cmd > 0
        cmd = torch.norm(env.command_manager.get_command("base_velocity"), dim=1)
        body_vel = torch.linalg.norm(self.asset.data.root_lin_vel_b[:, :2], dim=1)
        return torch.where(
            torch.logical_or(cmd > 0.0, body_vel > self.velocity_threshold), sync_reward * async_reward, 0.0
        )

    """
    Helper functions.
    """

    def _sync_reward_func(self, foot_0: int, foot_1: int) -> torch.Tensor:
        """Reward synchronization of two feet."""
        air_time = self.contact_sensor.data.current_air_time
        contact_time = self.contact_sensor.data.current_contact_time
        # penalize the difference between the most recent air time and contact time of synced feet pairs.
        se_air = torch.clip(torch.square(air_time[:, foot_0] - air_time[:, foot_1]), max=self.max_err**2)
        se_contact = torch.clip(torch.square(contact_time[:, foot_0] - contact_time[:, foot_1]), max=self.max_err**2)
        return torch.exp(-(se_air + se_contact) / self.std)

    def _async_reward_func(self, foot_0: int, foot_1: int) -> torch.Tensor:
        """Reward anti-synchronization of two feet."""
        air_time = self.contact_sensor.data.current_air_time
        contact_time = self.contact_sensor.data.current_contact_time
        # penalize the difference between opposing contact modes air time of feet 1 to contact time of feet 2
        # and contact time of feet 1 to air time of feet 2) of feet pairs that are not in sync with each other.
        se_act_0 = torch.clip(torch.square(air_time[:, foot_0] - contact_time[:, foot_1]), max=self.max_err**2)
        se_act_1 = torch.clip(torch.square(contact_time[:, foot_0] - air_time[:, foot_1]), max=self.max_err**2)
        return torch.exp(-(se_act_0 + se_act_1) / self.std)


def foot_clearance_reward(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, target_height: float, std: float, tanh_mult: float,
    lin_vel_threshold: float = 0.15,
) -> torch.Tensor:
    """Reward the swinging feet for clearing a specified height off the ground.

    Gated on any motion command (linear OR yaw) so feet lift during turning as well
    as during forward/lateral walking. Without the yaw term, foot_clearance was
    exactly zero during pure yaw commands, giving the robot no incentive to lift
    feet while spinning → heel-pivot / heel-strike during yaw turns.
    Yaw rate is scaled by 0.3 to convert rad/s → comparable magnitude to m/s.
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    foot_z_target_error = torch.square(asset.data.body_pos_w[:, asset_cfg.body_ids, 2] - target_height)
    foot_velocity_tanh = torch.tanh(tanh_mult * torch.norm(asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :2], dim=2))
    # Velocity gate OUTSIDE the exponent: stationary foot gets 0, not exp(0)=1
    per_foot_reward = torch.exp(-foot_z_target_error / std) * foot_velocity_tanh
    base_reward = torch.sum(per_foot_reward, dim=1)
    # Gate: active when robot has meaningful linear OR yaw velocity command
    vel_cmd = env.command_manager.get_command("base_velocity")
    lin_vel_cmd = torch.norm(vel_cmd[:, :2], dim=1)
    yaw_cmd = torch.abs(vel_cmd[:, 2]) * 0.3  # scale rad/s → comparable to m/s
    combined = torch.max(lin_vel_cmd, yaw_cmd)
    lin_scale = torch.clamp(combined / lin_vel_threshold, 0.0, 1.0)
    return base_reward * lin_scale


def ankle_angle_penalty(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, yaw_threshold: float = 0.1, lin_vel_threshold: float = 0.15,
) -> torch.Tensor:
    """Penalize ankle pitch away from 0 during yaw-dominant motion (turning in place).

    During pure yaw turns the ankle should stay flat — toe-lift causes balance issues.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    cmd = env.command_manager.get_command("base_velocity")
    lin_vel_cmd = torch.norm(cmd[:, :2], dim=1)
    yaw_cmd = torch.abs(cmd[:, 2])
    # Scale penalty by how yaw-dominant the command is (high yaw, low linear)
    yaw_scale = torch.clamp(yaw_cmd / yaw_threshold, 0.0, 1.0)
    lin_scale = 1.0 - torch.clamp(lin_vel_cmd / lin_vel_threshold, 0.0, 1.0)
    gate = yaw_scale * lin_scale
    ankle_deviation = torch.sum(torch.abs(asset.data.joint_pos[:, asset_cfg.joint_ids]), dim=1)
    return gate * ankle_deviation


def yaw_turn_foot_orientation_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    yaw_threshold: float = 0.3,
    lin_vel_threshold: float = 0.2,
) -> torch.Tensor:
    """Penalize foot pitch tilt in world space during yaw-dominant motion.

    Checks actual foot body orientation rather than ankle joint angle, so it
    catches heel-pivot even when the ankle joint reads near zero.
    Gated on high yaw command + low linear velocity (turning in place).
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    cmd = env.command_manager.get_command("base_velocity")
    lin_vel_cmd = torch.norm(cmd[:, :2], dim=1)
    yaw_cmd = torch.abs(cmd[:, 2])

    # Gate: active when yaw-dominant and low linear velocity
    yaw_scale = torch.clamp(yaw_cmd / yaw_threshold, 0.0, 1.0)
    lin_scale = 1.0 - torch.clamp(lin_vel_cmd / lin_vel_threshold, 0.0, 1.0)
    gate = yaw_scale * lin_scale  # (B,)

    # Foot quaternions in world frame: (B, num_feet, 4) — [w, x, y, z]
    foot_quats = asset.data.body_quat_w[:, asset_cfg.body_ids, :]  # (B, 2, 4)

    # Extract pitch tilt: project gravity into foot frame and measure xy deviation
    # Simpler: use the foot's local z-axis (3rd column of rotation matrix) vs world z
    # quat: [w, x, y, z]
    w = foot_quats[..., 0]
    x = foot_quats[..., 1]
    y = foot_quats[..., 2]
    z = foot_quats[..., 3]

    # Local z-axis of foot in world frame (3rd column of rotation matrix from quat)
    foot_z_world_x = 2.0 * (x * z + w * y)
    foot_z_world_y = 2.0 * (y * z - w * x)
    foot_z_world_z = 1.0 - 2.0 * (x * x + y * y)

    # Tilt = deviation of foot z-axis from world z-axis
    # Perfect flat foot: foot_z_world = [0, 0, 1], tilt_xy = [0, 0]
    tilt = torch.sqrt(foot_z_world_x ** 2 + foot_z_world_y ** 2 + 1e-6)  # (B, 2)

    penalty = gate.unsqueeze(1) * tilt
    return torch.sum(penalty, dim=1)


def stance_foot_flat_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    sensor_cfg: SceneEntityCfg,
    threshold: float = 1.0,
) -> torch.Tensor:
    """Penalize foot tilt in world space during stance (foot in contact with ground).

    Uses actual foot body orientation (world-frame quaternion) rather than ankle
    joint angle, so it directly catches heel-strike and toe-strike regardless of
    velocity command direction (forward, lateral, backward, yaw).

    This is the primary anti-heel-strike term. Unlike yaw_foot_flat which is gated
    to only fire during pure yaw turns, this fires whenever the foot is on the ground.

    A flat foot has its local z-axis pointing straight up (world +Z).
    Any tilt away from vertical produces a non-zero penalty.
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]

    # is_contact: (B, 2) — True when foot is in contact
    net_forces = contact_sensor.data.net_forces_w_history
    is_contact = torch.max(torch.norm(net_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1)[0] > threshold

    # Foot quaternions in world frame: (B, 2, 4) — [w, x, y, z]
    foot_quats = asset.data.body_quat_w[:, asset_cfg.body_ids, :]  # (B, 2, 4)

    w = foot_quats[..., 0]
    x = foot_quats[..., 1]
    y = foot_quats[..., 2]
    z = foot_quats[..., 3]

    # Local z-axis of foot expressed in world frame (3rd column of rotation matrix)
    # Perfect flat foot: this equals [0, 0, 1] → tilt = 0
    foot_z_world_x = 2.0 * (x * z + w * y)
    foot_z_world_y = 2.0 * (y * z - w * x)

    # Tilt magnitude: 0 when perfectly flat, ~1 at 90 degrees
    tilt = torch.sqrt(foot_z_world_x ** 2 + foot_z_world_y ** 2 + 1e-6)  # (B, 2)

    # Only penalize during stance — no velocity command gate
    penalty = is_contact.float() * tilt
    return torch.sum(penalty, dim=1)


##
# Regularization Penalties
##


def action_smoothness_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize large instantaneous changes in the network action output"""
    return torch.linalg.norm((env.action_manager.action - env.action_manager.prev_action), dim=1)


def air_time_variance_penalty(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize variance in the amount of time each foot spends in the air/on the ground relative to each other"""
    # extract the used quantities (to enable type-hinting)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    if contact_sensor.cfg.track_air_time is False:
        raise RuntimeError("Activate ContactSensor's track_air_time!")
    # compute the reward
    last_air_time = contact_sensor.data.last_air_time[:, sensor_cfg.body_ids]
    last_contact_time = contact_sensor.data.last_contact_time[:, sensor_cfg.body_ids]
    return torch.var(torch.clip(last_air_time, max=0.5), dim=1) + torch.var(
        torch.clip(last_contact_time, max=0.5), dim=1
    )


# ! look into simplifying the kernel here; it's a little oddly complex
def base_motion_penalty(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize base vertical and roll/pitch velocity"""
    # extract the used quantities (to enable type-hinting)
    asset: RigidObject = env.scene[asset_cfg.name]
    return 0.8 * torch.square(asset.data.root_lin_vel_b[:, 2]) + 0.2 * torch.sum(
        torch.abs(asset.data.root_ang_vel_b[:, :2]), dim=1
    )


def base_orientation_penalty(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize non-flat base orientation

    This is computed by penalizing the xy-components of the projected gravity vector.
    """
    # extract the used quantities (to enable type-hinting)
    asset: RigidObject = env.scene[asset_cfg.name]
    return torch.linalg.norm((asset.data.projected_gravity_b[:, :2]), dim=1)


def foot_slip_penalty(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, sensor_cfg: SceneEntityCfg, threshold: float
) -> torch.Tensor:
    """Penalize foot planar (xy) slip when in contact with the ground"""
    asset: RigidObject = env.scene[asset_cfg.name]
    # extract the used quantities (to enable type-hinting)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]

    # check if contact force is above threshold
    net_contact_forces = contact_sensor.data.net_forces_w_history
    is_contact = torch.max(torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1)[0] > threshold
    foot_planar_velocity = torch.linalg.norm(asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :2], dim=2)

    reward = is_contact * foot_planar_velocity
    return torch.sum(reward, dim=1)


def stance_ankle_deviation_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    sensor_cfg: SceneEntityCfg,
    threshold: float = 1.0,
) -> torch.Tensor:
    """Penalize ankle deviation from 0 during stance phase (foot on the ground).

    Prevents tiptoe walking by keeping the ankle near neutral during contact.
    Mirrors swing_ankle_deviation_penalty but fires when the foot IS in contact.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]

    net_forces = contact_sensor.data.net_forces_w_history
    is_contact = torch.max(torch.norm(net_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1)[0] > threshold

    ankle_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]  # (B, 2)
    penalty = is_contact.float() * torch.abs(ankle_pos)
    return torch.sum(penalty, dim=1)


def swing_ankle_deviation_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    sensor_cfg: SceneEntityCfg,
    threshold: float = 1.0,
) -> torch.Tensor:
    """Penalize ankle deviation from 0 during swing phase (foot in the air).

    During stance/toe-off the ankle is free to flex. During swing it should
    stay near neutral so the toe doesn't drag on the ground.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]

    # is_contact: (B, 2) — True when foot is on the ground
    net_forces = contact_sensor.data.net_forces_w_history
    is_contact = torch.max(torch.norm(net_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1)[0] > threshold
    is_swing = ~is_contact  # (B, 2)

    # ankle_pos: (B, 2) — left then right ankle deviation from 0
    ankle_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]  # (B, 2)

    # Penalize any ankle deviation from neutral during swing.
    penalty = is_swing.float() * torch.abs(ankle_pos)
    return torch.sum(penalty, dim=1)


def joint_torque_symmetry_penalty(
    env: ManagerBasedRLEnv,
    left_cfg: SceneEntityCfg,
    right_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Penalize torque asymmetry between corresponding left/right joint pairs.

    For each paired (left, right) joint, penalizes |torque_left - torque_right|.
    left_cfg and right_cfg must list the same number of joints in matching order.
    Forces both legs to share work equally rather than offloading to one side.
    """
    asset: Articulation = env.scene[left_cfg.name]
    left_t  = torch.abs(asset.data.applied_torque[:, left_cfg.joint_ids])   # (B, n)
    right_t = torch.abs(asset.data.applied_torque[:, right_cfg.joint_ids])  # (B, n)
    return torch.sum(torch.abs(left_t - right_t), dim=1)


def joint_velocity_penalty(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize high joint velocities."""
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.linalg.norm(asset.data.joint_vel[:, asset_cfg.joint_ids], dim=1)


def joint_position_penalty(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, stand_still_scale: float, velocity_threshold: float
) -> torch.Tensor:
    """Penalize joint position error from default on the articulation."""
    asset: Articulation = env.scene[asset_cfg.name]
    cmd = torch.linalg.norm(env.command_manager.get_command("base_velocity"), dim=1)
    body_vel = torch.linalg.norm(asset.data.root_lin_vel_b[:, :2], dim=1)
    reward = torch.linalg.norm((asset.data.joint_pos - asset.data.default_joint_pos), dim=1)
    return torch.where(torch.logical_or(cmd > 0.0, body_vel > velocity_threshold), reward, stand_still_scale * reward)


def foot_step_symmetry_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Penalize asymmetric foot reach in the sagittal (forward/backward) plane.

    For a symmetric bipedal gait, when the left foot is X meters ahead of the base,
    the right foot should be X meters behind — so their X offsets in body frame sum to ~0.
    A persistent non-zero sum means one foot consistently reaches further than the other.

    Penalty = |left_foot_x_body + right_foot_x_body|
    """
    asset: RigidObject = env.scene[asset_cfg.name]

    # Foot world positions: (B, 2, 3)
    foot_pos_w = asset.data.body_pos_w[:, asset_cfg.body_ids, :]
    base_pos_w = asset.data.root_pos_w          # (B, 3)
    base_quat_w = asset.data.root_quat_w        # (B, 4)

    B = foot_pos_w.shape[0]

    # Foot positions relative to base, in world frame: (B, 2, 3)
    rel_pos_w = foot_pos_w - base_pos_w.unsqueeze(1)

    # Rotate into base body frame: (B*2, 3)
    quat_exp = base_quat_w.unsqueeze(1).expand(-1, 2, -1).reshape(B * 2, 4)
    foot_pos_b = quat_apply_inverse(quat_exp, rel_pos_w.reshape(B * 2, 3)).reshape(B, 2, 3)

    # X = forward in body frame; left=[:,0], right=[:,1]
    foot_x = foot_pos_b[:, :, 0]  # (B, 2)

    # Symmetric walk: foot_x[:,0] + foot_x[:,1] ≈ 0
    # Clamp to 0.5m max to prevent large spikes during falls/pushes
    return torch.clamp(torch.abs(foot_x[:, 0] + foot_x[:, 1]), max=0.5)


##
# Motion Imitation
##


class ImitationMotion(ManagerTermBase):
    """Reward for imitating a polynomial reference walking motion.

    Combines three sub-rewards (weighted internally):
      - imitate_joint_pos: exp-kernel on joint position error vs reference
      - imitate_joint_vel: exp-kernel on joint velocity error vs reference
      - imitate_foot_contact: fraction of feet whose contact state matches reference

    Reference clips are selected per-env based on nearest velocity command.
    Gait phase advances by dt/period each step and resets to a random value
    on episode reset for training diversity.

    Reference joint order (10-joint BDX-R pkl):
      0 L_HipYaw  1 L_HipRoll  2 L_HipPitch  3 L_Knee  4 L_Ankle
      5 R_HipYaw  6 R_HipRoll  7 R_HipPitch  8 R_Knee  9 R_Ankle
    Floyd uses indices [1,2,3,4,6,7,8,9] (skip yaw at 0 and 5).
    Floyd joint order assumed: left_hip_roll, left_hip_pitch, left_knee, left_ankle,
                               right_hip_roll, right_hip_pitch, right_knee, right_ankle
    """

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)

        pkl_path         = cfg.params["pkl_path"]
        self._std_jpos   = cfg.params.get("std_jpos",           0.25)
        self._std_jvel   = cfg.params.get("std_jvel",           2.0)
        self._w_jpos     = cfg.params.get("w_jpos",             1.5)
        self._w_jvel     = cfg.params.get("w_jvel",             0.1)
        self._w_contact  = cfg.params.get("w_contact",          0.5)
        self._threshold  = cfg.params.get("contact_threshold",  1.0)

        # Reference motion library
        self._library = ReferenceMotionLibraryLegs(pkl_path, device=env.device)
        self._ref_idx = torch.tensor(FLOYD_REF_INDICES, dtype=torch.long, device=env.device)

        # Floyd joint IDs in same order as FLOYD_REF_INDICES mapping
        floyd_joint_names = [
            "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
            "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
        ]
        robot = env.scene["robot"]
        jn = robot.data.joint_names
        self._joint_ids = torch.tensor(
            [jn.index(n) for n in floyd_joint_names], dtype=torch.long, device=env.device
        )

        # Contact sensor and foot body IDs [left, right]
        sensor_name  = cfg.params.get("sensor_name",  "contact_forces")
        foot_names   = cfg.params.get("foot_names",   ["FootBaseLeft", "FootBaseRight"])
        self._contact_sensor: ContactSensor = env.scene.sensors[sensor_name]
        self._foot_ids, _  = self._contact_sensor.find_bodies(foot_names)

        # Per-env phase buffer and clip selection
        self._phase     = torch.zeros(env.num_envs, device=env.device)
        self._clip_idx  = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)

        # Sign correction for right hip pitch.
        #
        # Floyd jpos_ref layout after FLOYD_REF_INDICES:
        #   [0] L_HipRoll  [1] L_HipPitch  [2] L_Knee  [3] L_Ankle
        #   [4] R_HipRoll  [5] R_HipPitch  [6] R_Knee  [7] R_Ankle
        #
        # Verified by tracing the full URDF kinematic chain (all fixed joints included)
        # to compute world-frame rotation axes for each joint:
        #
        #   joint           Floyd world axis    BDX-R limits
        #   L_HipPitch      [0, +1, 0]          (-0.75, 0.70)  ← positive = backward
        #   R_HipPitch      [0, -1, 0]          (-0.75, 0.70)  ← positive = FORWARD
        #   L_Knee          [0,-0.766,+0.643]   (-0.94,  1.30) ← mirrored limits
        #   R_Knee          [0,+0.766,-0.643]   (-1.30,  0.94) ← OPPOSITE axis = match
        #   L_Ankle         [0,-0.766,+0.643]   (-0.84,  1.20)
        #   R_Ankle         [0,+0.766,-0.643]   (-1.20,  0.84) ← OPPOSITE axis = match
        #
        # Hip pitch: BDX-R uses identical limits for L and R (same sign convention —
        # positive = backward extension for both sides).  Floyd's L and R hip pitch
        # axes are OPPOSITE ([0,+1,0] vs [0,-1,0]), so positive L = backward while
        # positive R = forward.  The left side therefore already matches BDX-R; only
        # R_HipPitch (index 5) must be negated so that the reference correctly drives
        # Floyd's right leg backward during stance / forward during swing.
        #
        # Knee and ankle: Floyd's axes ARE already mirrored between sides (OPPOSITE),
        # which naturally matches BDX-R's mirrored limits — no sign flip needed.
        self._ref_sign = torch.ones(8, device=env.device)
        self._ref_sign[5] = -1.  # R_HipPitch: BDX-R positive=backward, Floyd positive=forward

        # Cache on env so observation functions can read without re-computing
        env._imitation_phase    = torch.zeros(env.num_envs, 1, device=env.device)
        env._imitation_ref_jpos = torch.zeros(env.num_envs, 8, device=env.device)

    def reset(self, env_ids: torch.Tensor) -> None:
        """Randomise starting phase so the policy sees diverse gait states."""
        self._phase[env_ids] = torch.rand(len(env_ids), device=self._phase.device)

    def __call__(
        self,
        env: ManagerBasedRLEnv,
        pkl_path: str,
        std_jpos: float          = 0.25,
        std_jvel: float          = 2.0,
        w_jpos: float            = 1.5,
        w_jvel: float            = 0.1,
        w_contact: float         = 0.5,
        contact_threshold: float = 1.0,
        sensor_name: str         = "contact_forces",
        foot_names               = None,
    ) -> torch.Tensor:
        robot: Articulation = env.scene["robot"]

        # Select nearest reference clip by velocity command [vx, vy, 0]
        vel_cmd = env.command_manager.get_command("base_velocity")
        vel_3d  = torch.cat(
            [vel_cmd[:, :2], torch.zeros(env.num_envs, 1, device=env.device)], dim=1
        )
        self._clip_idx = self._library.nearest_clip(vel_3d)

        # Advance phase by dt/period, wrap at 1.0
        period       = self._library.period(self._clip_idx)
        self._phase  = (self._phase + env.step_dt / period) % 1.0

        # Query reference joint positions, velocities, and foot contacts
        _, _, jpos_ref10, jvel_ref10, foot_ref = self._library.query(self._clip_idx, self._phase)
        jpos_ref = jpos_ref10[:, self._ref_idx] * self._ref_sign   # (B, 8)
        jvel_ref = jvel_ref10[:, self._ref_idx] * self._ref_sign   # (B, 8)

        # Cache for observation functions (one-step lag is acceptable)
        env._imitation_phase    = self._phase.unsqueeze(1).detach()
        env._imitation_ref_jpos = jpos_ref.detach()

        # ── Joint position reward ────────────────────────────────────────
        jpos     = robot.data.joint_pos[:, self._joint_ids]
        jpos_err = torch.sum(torch.square(jpos - jpos_ref), dim=1)
        r_jpos   = torch.exp(-jpos_err / (self._std_jpos ** 2)) * self._w_jpos

        # ── Joint velocity reward ────────────────────────────────────────
        jvel     = robot.data.joint_vel[:, self._joint_ids]
        jvel_err = torch.sum(torch.square(jvel - jvel_ref), dim=1)
        r_jvel   = torch.exp(-jvel_err / (self._std_jvel ** 2)) * self._w_jvel

        # ── Foot contact reward ──────────────────────────────────────────
        net_forces = self._contact_sensor.data.net_forces_w_history
        is_contact = (
            torch.max(torch.norm(net_forces[:, :, self._foot_ids], dim=-1), dim=1)[0]
            > self._threshold
        )  # (B, 2)
        ref_contact = foot_ref > 0.5          # (B, 2) — threshold polynomial value
        match       = (is_contact == ref_contact).float()
        r_contact   = match.mean(dim=1) * self._w_contact

        return r_jpos + r_jvel + r_contact


##
# Observation helpers for imitation (read cached state written by ImitationMotion)
##


def imitation_phase(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Current gait phase in [0, 1). Updated by ImitationMotion reward term."""
    return getattr(env, "_imitation_phase",
                   torch.zeros(env.num_envs, 1, device=env.device))


def ref_joint_pos(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Reference joint positions (8 DOF) at current phase. Updated by ImitationMotion."""
    return getattr(env, "_imitation_ref_jpos",
                   torch.zeros(env.num_envs, 8, device=env.device))
