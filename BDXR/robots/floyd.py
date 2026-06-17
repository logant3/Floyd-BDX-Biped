from pathlib import Path
import isaaclab.sim as sim_utils
from isaaclab.actuators import DelayedPDActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

##
# Configuration
##

FLOYD_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=str(Path(__file__).parent.parent.parent / "robot_description" / "urdf" / "Floyd_URDF.urdf"),
        fix_base=False,
        merge_fixed_joints=True,
        activate_contact_sensors=True,
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.13),
        joint_pos={
            "LeftHipRoll": 0.0,
            "RightHipRoll": 0.0,
            "LeftHipPitch": 0.0,
            "RightHipPitch": 0.0,
            "LeftKneePitch": 0.0,
            "RightKneePitch": 0.0,
            "LeftAnklePitch": 0.0,
            "RightAnklePitch": 0.0,
        },
    ),
    actuators={
        "legs_RS03": DelayedPDActuatorCfg(
            joint_names_expr=[".*HipRoll", ".*HipPitch", ".*KneePitch"],
            stiffness=15.92966,
            damping=0.00637,
            armature=0.02,
            effort_limit_sim=60.0,
            velocity_limit_sim=18.849,
            min_delay=0,
            max_delay=0,
        ),
        "legs_RS02": DelayedPDActuatorCfg(
            joint_names_expr=[".*AnklePitch"],
            stiffness=3.34728,
            damping=0.00134,
            armature=0.0042,
            effort_limit_sim=17.0,
            velocity_limit_sim=37.699,
            min_delay=0,
            max_delay=0,
        ),
    },
    soft_joint_pos_limit_factor=0.85,
)
"""Configuration for the Floyd BDX-style biped robot."""
