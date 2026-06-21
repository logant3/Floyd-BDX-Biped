import gymnasium as gym
from . import agents
from .il_work.floyd_env_cfg_il import FloydEnvCfg_IL, FloydEnvCfg_IL_PLAY

gym.register(
    id='Floyd-Velocity-Flat-IL-v0',
    entry_point='isaaclab.envs:ManagerBasedRLEnv',
    disable_env_checker=True,
    kwargs={
        'env_cfg_entry_point': f'{__name__}.il_work.floyd_env_cfg_il:FloydEnvCfg_IL',
        'rsl_rl_cfg_entry_point': f'{agents.__name__}.rsl_rl_ppo_cfg:PPORunnerCfg',
    },
)

gym.register(
    id='Floyd-Velocity-Flat-IL-Play-v0',
    entry_point='isaaclab.envs:ManagerBasedRLEnv',
    disable_env_checker=True,
    kwargs={
        'env_cfg_entry_point': f'{__name__}.il_work.floyd_env_cfg_il:FloydEnvCfg_IL_PLAY',
        'rsl_rl_cfg_entry_point': f'{agents.__name__}.rsl_rl_ppo_cfg:PPORunnerCfg',
    },
)

gym.register(
    id='Floyd-Velocity-Flat-v0',
    entry_point='isaaclab.envs:ManagerBasedRLEnv',
    disable_env_checker=True,
    kwargs={
        'env_cfg_entry_point': f'{__name__}.floyd_env_cfg:FloydEnvCfg',
        'rsl_rl_cfg_entry_point': f'{agents.__name__}.rsl_rl_ppo_cfg:PPORunnerCfg',
    },
)

gym.register(
    id='Floyd-Velocity-Flat-Play-v0',
    entry_point='isaaclab.envs:ManagerBasedRLEnv',
    disable_env_checker=True,
    kwargs={
        'env_cfg_entry_point': f'{__name__}.floyd_env_cfg:FloydEnvCfg_PLAY',
        'rsl_rl_cfg_entry_point': f'{agents.__name__}.rsl_rl_ppo_cfg:PPORunnerCfg',
    },
)
