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
from envs import get_env
from controllers import get_controller

def run(cfg):
    # get env
    env = get_env(cfg['env'])

    # set GP control params
    ctrl_cfg = cfg['controller']
    ctrl_cfg_params = ctrl_cfg['params']
    env.sim.control_dt = ctrl_cfg_params['control_dt']
    if ctrl_cfg_params['hand_control_mode'] == "position":
        env.sim.hand_control_mode = bg.HandControlMode.POSITION_CONTROL
    elif ctrl_cfg_params['hand_control_mode'] == "current":
        env.sim.hand_control_mode = bg.HandControlMode.CURRENT_CONTROL

    # get controller
    controller = get_controller(ctrl_cfg)

    atexit.register(env.sim.shutdown)
    print("Finished init.")

    # start experiment
    try:
        tty.setcbreak(sys.stdin.fileno())
        env.sim.initialize() # TODO: make sure that this waits for gripper to be initialized
        controller.begin(env.sim)
        env.sim.apply_control()
        env.sim.sync_viewer()
        print("Starting main loop.")

        import time
        tic = time.time()

        while env.sim.mode==bg.PlatformMode.HW_NO_VIS or env.sim.mj_viewer.is_running(): # TODO: better way to do this?
            if not env.sim.paused:
                # step in time to update data from hardware or sim
                env.sim.step()
                # run controller and update commands
                env.sim.dt_comp = 0.0 # for real-time simulation
                if env.sim.run_control:
                    control_start_time = env.sim.time()
                    env.sim.run_control = False
                    env.sim.sync_data()
                    controller.update(env.sim)
                    env.sim.apply_control()
                    env.sim.log_data()
                    env.sim.dt_comp += env.sim.time() - control_start_time
                # sync viewer
                if env.sim.run_viewer_sync:
                    viewer_sync_start_time = env.sim.time()
                    env.sim.run_viewer_sync = False
                    env.sim.sync_viewer()
                    env.sim.dt_comp += env.sim.time() - viewer_sync_start_time

            ## added for time varying env
            env.update(time.time() - tic)

    # end experiment
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, env.sim.init_settings)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str)
    args, unknown = parser.parse_known_args()

    cfg = OmegaConf.load(args.config)
    print(OmegaConf.to_yaml(cfg))

    run(cfg)