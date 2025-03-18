# imports
import brl_gripper as bg
import mujoco as mj
import atexit
import tty
import termios
import sys
import os

import argparse
from omegaconf import OmegaConf
from controllers import get_controller

def run(cfg):
    # initialization
    print("Starting init.")
    init_settings = termios.tcgetattr(sys.stdin)

    # platform
    xml_path = os.path.join(bg.assets.ASSETS_DIR, cfg['xml_scene'])
    log_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),'logs/')

    hw_mode = bg.HardwareEnable.NO_HW
    mj_model = mj.MjModel.from_xml_path(xml_path+".xml")

    if cfg['gp_type'] == 'v1':
        GP = bg.GripperPlatform(mj_model, viewer_enable=True, hardware_enable=hw_mode, log_path=None)
    elif cfg['gp_type'] == 'v2':
        GP = bg.GripperPlatformV2(mj_model, viewer_enable=True, hardware_enable=hw_mode, log_path=None)
    
    # set GP control params
    ctrl_cfg = cfg['controller']
    ctrl_cfg_params = ctrl_cfg['params']
    GP.control_dt = ctrl_cfg_params['control_dt']
    if ctrl_cfg_params['hand_control_mode'] == "position":
        GP.hand_control_mode = bg.HandControlMode.POSITION_CONTROL
    elif ctrl_cfg_params['hand_control_mode'] == "current":
        GP.hand_control_mode = bg.HandControlMode.CURRENT_CONTROL

    # get controller
    controller = get_controller(ctrl_cfg)

    atexit.register(GP.shutdown)
    print("Finished init.")

    # start experiment
    try:
        tty.setcbreak(sys.stdin.fileno())
        GP.initialize() # TODO: make sure that this waits for gripper to be initialized
        controller.begin(GP.gr_data)
        GP.apply_control()
        GP.sync_viewer()
        print("Starting main loop.")
        while GP.mode==bg.PlatformMode.HW_NO_VIS or GP.mj_viewer.is_running(): # TODO: better way to do this?
            if not GP.paused:
                # step in time to update data from hardware or sim
                GP.step()
                # run controller and update commands
                GP.dt_comp = 0.0 # for real-time simulation
                if GP.run_control:
                    control_start_time = GP.time()
                    GP.run_control = False
                    GP.sync_data()
                    controller.update(GP.gr_data)
                    GP.apply_control()
                    GP.log_data()
                    GP.dt_comp += GP.time() - control_start_time
                # sync viewer
                if GP.run_viewer_sync:
                    viewer_sync_start_time = GP.time()
                    GP.run_viewer_sync = False
                    GP.sync_viewer()
                    GP.dt_comp += GP.time() - viewer_sync_start_time

    # end experiment
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, init_settings)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str)
    args, unknown = parser.parse_known_args()

    cfg = OmegaConf.load(args.config)
    print(OmegaConf.to_yaml(cfg))

    run(cfg)