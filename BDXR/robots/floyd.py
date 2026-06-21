from pathlib import Path
import isaaclab.sim as sim_utils
from isaaclab.actuators import DelayedPDActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

##
# Configuration
##

FLOYD_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path="C:/Users/logan/Floyd-IsaacLab/BDXR/robots/urdf/Floyd_URDF.urdf",
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
        pos=(0.0, 0.0, 0.12),
        joint_pos={
            "left_hip_roll": 0.0,
            "right_hip_roll": 0.0,
            "left_hip_pitch": 0.0,
            "right_hip_pitch": 0.0,
            "left_knee": 0.0,
            "right_knee": 0.0,
            "left_ankle": 0.0,
            "right_ankle": 0.0,
        },
    ),
    actuators={
        # RS03 joints — exact gains from Kayden's policy.yaml and config.py
        "legs_RS03": DelayedPDActuatorCfg(
            joint_names_expr=[".*hip_roll", ".*hip_pitch", ".*knee"],
            stiffness=78.957,
            damping=5.027,
            armature=0.02,
            effort_limit_sim=60.0,
            velocity_limit_sim=18.849,
            min_delay=0,
            max_delay=0,
        ),
        # RS02 joints — exact gains from Kayden's policy.yaml and config.py
        "legs_RS02": DelayedPDActuatorCfg(
            joint_names_expr=[".*ankle"],
            stiffness=16.581,
            damping=1.056,
            armature=0.0042,
            effort_limit_sim=17.0,
            velocity_limit_sim=37.699,
            min_delay=0,
            max_delay=0,
        ),
    },
    soft_joint_pos_limit_factor=0.95,
)
"""Configuration for the Floyd BDX-style biped robot."""