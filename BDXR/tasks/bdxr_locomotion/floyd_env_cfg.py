import math
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
import isaaclab.sim as sim_utils

from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg, RewardsCfg
import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab.managers import TerminationTermCfg as DoneTerm

from BDXR.robots.floyd import FLOYD_CFG  # isort:skip

# After URDF import with merge_fixed_joints=True:
# FootCoverRight + FootPadRight merged into FootBaseRight
# base_link merged into world
FOOT_BODIES = ["FootPadLeft", "FootBaseRight"]
BASE_BODY = "world"

@configclass
class FloydRewards(RewardsCfg):
    """Simplified reward set — closer to working disney_bdx reference."""

    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)

    feet_air_time = RewTerm(
        func=mdp.feet_air_time_positive_biped,
        weight=6.0,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_BODIES),
            "command_name": "base_velocity",
            "threshold": 0.3,
        },
    )

    dof_pos_limits = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*AnklePitch")},
    )

    joint_deviation_hip = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*HipRoll"])},
    )

    flat_orientation_l2 = RewTerm(
        func=mdp.flat_orientation_l2,
        weight=-5.0,
    )


@configclass
class FloydEnvCfg(LocomotionVelocityRoughEnvCfg):
    rewards: FloydRewards = FloydRewards()

    def __post_init__(self):
        super().__post_init__()

        # Robot
        self.scene.robot = FLOYD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # No height scanner — simpler to start
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

        # Rewards
        self.rewards.undesired_contacts = None
        self.rewards.dof_torques_l2.weight = -5.0e-6
        self.rewards.track_lin_vel_xy_exp.weight = 2.0
        self.rewards.track_ang_vel_z_exp.weight = 1.0
        self.rewards.action_rate_l2.weight *= 1.5
        self.rewards.dof_acc_l2.weight *= 1.5
        self.rewards.lin_vel_z_l2 = None
        self.rewards.ang_vel_xy_l2 = None

        # Commands — start with just forward walking
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)


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
