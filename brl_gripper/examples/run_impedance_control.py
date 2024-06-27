# imports
import brl_gripper as bg
import atexit
import tty
import termios
import sys
import os

# initialization
print("Starting init.")
init_settings = termios.tcgetattr(sys.stdin)

# platform
xml_path = os.path.join(bg.assets.ASSETS_DIR, 'scene')
log_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),'logs/')
GP = bg.GripperPlatform(xml_path, viewer_enable=True, hardware_enable=bg.NO_HW, log_path=log_path)

# controller
from controllers.finger_demos.impedance_control import ImpedanceControlDemo
controller = ImpedanceControlDemo()

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
    while GP.mode==bg.HW_NO_VIS or GP.mj_viewer.is_running(): # TODO: better way to do this?
        # step in time to update data from hardware or sim
        GP.step()
        # run controller and update commands
        if GP.run_control:
            GP.run_control = False
            GP.sync_data()
            controller.update(GP.gr_data)
            GP.apply_control()
            GP.log_data()
        # sync viewer
        if GP.run_viewer_sync:
            GP.run_viewer_sync = False
            GP.sync_viewer()

# end experiment
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, init_settings)