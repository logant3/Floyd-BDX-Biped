import math
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
import isaaclab.sim as sim_utils

from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg, RewardsCfg
import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab.managers import TerminationTermCfg as DoneTerm

from BDXR.robots.floyd import FLOYD_CFG  # isort:skip
import BDXR.tasks.bdxr_locomotion.mdp as floyd_mdp  # isort:skip

# After URDF import with merge_fixed_joints=True:
# All fixed joints merge into their revolute parents
# Contact bodies: FootPadLeft, FootPadRight (terminal foot links)
# Base body: world (base_link + all torso fixed links merge into world)
FOOT_BODIES = ["FootPadLeft", "FootPadRight"]
BASE_BODY = "world"


@configclass
class FloydRewards(RewardsCfg):
    """Reward terms — based on Kayden's BDXR config adapted for 8-DOF Floyd."""

    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)

    # Bipedal air time reward — encourages alternating foot contact
    air_time = RewTerm(
        func=floyd_mdp.bipedal_air_time_reward,
        weight=5.0,
        params={
            "mode_time": 0.3,
            "velocity_threshold": 0.5,
            "asset_cfg": SceneEntityCfg("robot"),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_BODIES),
        },
    )

    # Penalize ankle joint limits
    dof_pos_limits = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*AnklePitch")},
    )

    # Penalize hip roll deviation — keep hips centered
    joint_deviation_hip = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*HipRoll"])},
    )

    # Foot clearance — encourage lifting feet during swing
    foot_clearance = RewTerm(
        func=floyd_mdp.foot_clearance_reward,
        weight=2.0,
        params={
            "std": 0.05,
            "tanh_mult": 2.0,
            "target_height": 0.1,
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_BODIES),
        },
    )

    # Foot slip penalty
    foot_slip = RewTerm(
        func=floyd_mdp.foot_slip_penalty,
        weight=-0.5,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_BODIES),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_BODIES),
            "threshold": 1.0,
        },
    )

    # Strong orientation penalty to keep robot upright
    flat_orientation_l2 = RewTerm(
        func=mdp.flat_orientation_l2,
        weight=-10.0,
    )

    # Joint position penalty — stay near default pose when standing still
    joint_pos = RewTerm(
        func=floyd_mdp.joint_position_penalty,
        weight=-1.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "stand_still_scale": 5.0,
            "velocity_threshold": 0.5,
        },
    )

    # General joint velocity penalty — discourages fast flailing motions
    joint_vel_penalty = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.001,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"])},
    )

    # Extra penalty on hips and knees specifically — stops the wild leg swings
    hip_knee_vel_penalty = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.005,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*HipPitch", ".*KneePitch"])},
    )


@configclass
class FloydEnvCfg(LocomotionVelocityRoughEnvCfg):
    rewards: FloydRewards = FloydRewards()

    def __post_init__(self):
        super().__post_init__()

        # Robot
        self.scene.robot = FLOYD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # No height scanner — keep it simple
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None

        # Flat terrain only
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        self.curriculum.terrain_levels = None

        # Actions
        self.actions.joint_pos.scale = 0.5

        # Events
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

        # Terminations
        self.terminations.base_contact.params["sensor_cfg"].body_names = [BASE_BODY]

        # Rewards from parent — tune weights
        self.rewards.undesired_contacts = None
        self.rewards.feet_air_time = None
        self.rewards.dof_torques_l2.weight = -5.0e-6
        self.rewards.track_lin_vel_xy_exp.weight = 5.0
        self.rewards.track_ang_vel_z_exp.weight = 2.0
        self.rewards.action_rate_l2.weight = -0.05
        self.rewards.dof_acc_l2.weight = -1.25e-7
        self.rewards.lin_vel_z_l2.weight = -2.0
        self.rewards.ang_vel_xy_l2.weight = -0.05

        # Forward only — no sideways, no turning to start
        self.commands.base_velocity.heading_command = False
        self.commands.base_velocity.ranges.lin_vel_x = (0.3, 0.8)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)


@configclass
class FloydEnvCfg_PLAY(FloydEnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.commands.base_velocity.ranges.lin_vel_x = (0.5, 0.5)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)