from envs.base import BaseEnv
from envs.grasping import TopDownGraspingEnv

def get_env(cfg_env):
    name = cfg_env['name']
    if name == 'base':
        return BaseEnv(**cfg_env)
    elif name == 'topdown_grasping':
        return TopDownGraspingEnv(**cfg_env)
    else:
        raise ValueError(f"Environment {name} not found.")