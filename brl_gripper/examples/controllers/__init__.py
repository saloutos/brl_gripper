from .finger_demos.position_trajectories import PosTrajDemo
from .finger_demos.velocity_field_control import VelocityFieldControlDemo
from .finger_demos.task_space_velocity_field_control import TaskSpaceVelocityFieldControlDemo

def get_controller(controller_cfg, *args, **kwargs):
    name = controller_cfg["name"]
    ctrl_class = _get_ctrl_instance(name)
    controller = ctrl_class(controller_cfg, *args, **kwargs)
    return controller

def _get_ctrl_instance(name):
    try:
        return {
            "postrajdemo": get_demo_controller,
            "vfcdemo": get_demo_controller,
            "tsvfcdemo": get_demo_controller,
        }[name]
    except:
        raise ("Model {} not available".format(name))
    
def get_demo_controller(controller_cfg, *args, **kwargs):
    name = controller_cfg['name']
    if name == 'postrajdemo':
        return PosTrajDemo()
    elif name == 'vfcdemo':
        return VelocityFieldControlDemo()
    elif name == 'tsvfcdemo':
        return TaskSpaceVelocityFieldControlDemo()
    else:
        raise ValueError(f"Controller {name} not found.")
    