import math
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg, RewardsCfg
from isaaclab_tasks.manager_based.locomotion.velocity.mdp import UniformVelocityCommandCfg

from BDXR.robots.floyd import FLOYD_CFG  # isort:skip
import BDXR.tasks.bdxr_locomotion.mdp as floyd_mdp  # isort:skip

FOOT_BODIES = ["FootBaseLeft", "FootBaseRight"]
BASE_BODY = "world"


@configclass
class FloydRewards(RewardsCfg):

    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)

    air_time = RewTerm(
        func=floyd_mdp.bipedal_air_time_reward,
        weight=5.0,
        params={
            "mode_time": 0.15,
            "velocity_threshold": 0.5,
            "asset_cfg": SceneEntityCfg("robot"),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_BODIES),
        },
    )


    dof_pos_limits = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*AnklePitch")},
    )

    joint_deviation_hip = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.5,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*HipRoll"])},
    )

    foot_clearance = RewTerm(
        func=floyd_mdp.foot_clearance_reward,
        weight=1.0,
        params={
            "std": 0.05,
            "tanh_mult": 2.0,
            "target_height": 0.15,
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_BODIES),
        },
    )

    both_feet_air = RewTerm(
        func=floyd_mdp.both_feet_air_penalty,
        weight=-3.0,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_BODIES),
            "threshold": 1.0,
        },
    )

    air_time_variance = RewTerm(
        func=floyd_mdp.air_time_variance_penalty,
        weight=-5.0,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_BODIES)},
    )

    foot_slip = RewTerm(
        func=floyd_mdp.foot_slip_penalty,
        weight=-0.5,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_BODIES),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_BODIES),
            "threshold": 1.0,
        },
    )

    joint_velocity = RewTerm(
        func=floyd_mdp.joint_velocity_penalty,
        weight=-0.05,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*")},
    )

    joint_pos = RewTerm(
        func=floyd_mdp.joint_position_penalty,
        weight=-1.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
            "stand_still_scale": 5.0,
            "velocity_threshold": 0.5,
        },
    )



@configclass
class FloydEnvCfg(LocomotionVelocityRoughEnvCfg):
    rewards: FloydRewards = FloydRewards()

    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = FLOYD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        self.scene.height_scanner = None
        self.observations.policy.height_scan = None

        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        self.curriculum.terrain_levels = None

        self.actions.joint_pos.scale = 0.5

        # Critical for sim-to-real: match deployment obs scaling in Kayden's config.py
        self.observations.policy.base_ang_vel.scale = 0.2
        self.observations.policy.joint_vel.scale = 0.05

        self.events.push_robot = None
        self.events.add_base_mass.params["asset_cfg"].body_names = [BASE_BODY]
        self.events.add_base_mass.params["mass_distribution_params"] = (-0.2, 0.5)
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.events.base_external_force_torque.params["asset_cfg"].body_names = [BASE_BODY]
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0),
                "roll": (0.0, 0.0), "pitch": (0.0, 0.0), "yaw": (0.0, 0.0),
            },
        }

        if hasattr(self.events, 'base_com'):
            self.events.base_com = None

        self.terminations.base_contact.params["sensor_cfg"].body_names = [BASE_BODY]
        self.terminations.base_contact.params["threshold"] = 1.0

        self.rewards.undesired_contacts = None
        self.rewards.feet_air_time = None
        self.rewards.dof_torques_l2.weight = -5.0e-6
        self.rewards.track_lin_vel_xy_exp.weight = 5.0
        self.rewards.track_ang_vel_z_exp.weight = 5.0
        self.rewards.action_rate_l2.weight = -0.05
        self.rewards.dof_acc_l2.weight = -1.25e-7
        self.rewards.lin_vel_z_l2.weight = -2.0
        self.rewards.ang_vel_xy_l2.weight = -0.05
        self.rewards.flat_orientation_l2.weight = -5.0

        self.commands.base_velocity = UniformVelocityCommandCfg(
            asset_name="robot",
            resampling_time_range=(10.0, 10.0),
            rel_standing_envs=0.02,
            rel_heading_envs=0.0,
            heading_command=True,
            debug_vis=True,
            ranges=UniformVelocityCommandCfg.Ranges(
                lin_vel_x=(-0.3, 0.5),
                lin_vel_y=(-0.4, 0.4),
                ang_vel_z=(-1.0, 1.0),
                heading=(-math.pi, math.pi),
            ),
        )


@configclass
class FloydEnvCfg_PLAY(FloydEnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.commands.base_velocity.ranges.lin_vel_x = (-0.3, 0.5)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.4, 0.4)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)