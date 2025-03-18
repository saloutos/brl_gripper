from controllers.tsvfc import (
    TaskSpaceVelocityFieldControllerDemo,
    TSVF_TD_Grasping
)

from controllers.vfc import GraspingVelocityFieldController

def get_controller(cfg):
    name = cfg['name']
    if name == "tsvfc_demo":
        return TaskSpaceVelocityFieldControllerDemo(**cfg)
    elif name == 'tsvfc_td_grasping':
        return TSVF_TD_Grasping(**cfg)
    elif name == 'vfc_grasping':
        return GraspingVelocityFieldController(**cfg)
    else:
        raise ValueError(f"Controller {name} not recognized.")