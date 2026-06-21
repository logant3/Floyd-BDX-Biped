"""Imitation Learning environment config for Floyd BDX-R.

Extends FloydEnvCfg with:
  - ImitationCommandFloyd term (phase tracking + polynomial reference)
  - phase + ref_joint_pos observations
  - imitate_joint_pos / imitate_joint_vel / imitate_foot_contact rewards

To use:
  Train:  python scripts/train.py --task Floyd-Velocity-Flat-IL-v0
  Play:   python scripts/train.py --task Floyd-Velocity-Flat-IL-Play-v0

To revert to pure RL (no IL), simply use Floyd-Velocity-Flat-v0 as before.

Generate polynomial first (run from Open_Duck_reference_motion_generator/):
  python scripts/generate_floyd_gaits.py --num 200 --output_dir recordings
  python scripts/fit_poly.py --ref_motion recordings
  (copy polynomial_coefficients.pkl to BDXR/tasks/bdxr_locomotion/)
"""

from pathlib import Path

from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from ..floyd_env_cfg import FloydEnvCfg, FloydEnvCfg_PLAY
from .mdp.imitation_command import ImitationCommandFloydCfg
from .mdp.imitation_observations import imitation_phase, ref_joint_pos
from .mdp.imitation_rewards import imitate_joint_pos, imitate_joint_vel, imitate_foot_contact

_DEFAULT_PKL = str(
    Path(__file__).parent / "polynomial_coefficients.pkl"
)


@configclass
class FloydEnvCfg_IL(FloydEnvCfg):
    """Floyd locomotion env with imitation learning rewards."""

    def __post_init__(self):
        super().__post_init__()

        # ── Imitation command (gait phase + reference joints) ─────────────────
        setattr(self.commands, "imitation", ImitationCommandFloydCfg(
            pkl_path=_DEFAULT_PKL,
        ))

        # ── Observations: add phase and reference joint positions ─────────────
        setattr(self.observations.policy, "imitation_phase", ObsTerm(
            func=imitation_phase,
            params={"command_name": "imitation"},
        ))
        setattr(self.observations.policy, "ref_joint_pos", ObsTerm(
            func=ref_joint_pos,
            params={"command_name": "imitation"},
        ))
        if hasattr(self.observations, "critic"):
            setattr(self.observations.critic, "imitation_phase", ObsTerm(
                func=imitation_phase,
                params={"command_name": "imitation"},
            ))
            setattr(self.observations.critic, "ref_joint_pos", ObsTerm(
                func=ref_joint_pos,
                params={"command_name": "imitation"},
            ))

        # ── Imitation rewards ─────────────────────────────────────────────────
        setattr(self.rewards, "imitate_joint_pos", RewTerm(
            func=imitate_joint_pos,
            weight=1.5,
            params={
                "std": 0.25,
                "command_name": "imitation",
                "asset_cfg": SceneEntityCfg("robot"),
            },
        ))
        setattr(self.rewards, "imitate_joint_vel", RewTerm(
            func=imitate_joint_vel,
            weight=0.1,
            params={
                "std": 2.0,
                "command_name": "imitation",
                "asset_cfg": SceneEntityCfg("robot"),
            },
        ))
        setattr(self.rewards, "imitate_foot_contact", RewTerm(
            func=imitate_foot_contact,
            weight=0.5,
            params={
                "command_name": "imitation",
                "threshold": 1.0,
                "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["FootBaseLeft", "FootBaseRight"]),
            },
        ))


@configclass
class FloydEnvCfg_IL_PLAY(FloydEnvCfg_IL):
    def __post_init__(self) -> None:
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
