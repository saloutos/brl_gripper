# imports
import brl_gripper as bg
import mujoco as mj
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

hw_mode = bg.HardwareEnable.NO_HW
if hw_mode == bg.HardwareEnable.NO_HW:
    mj_model = mj.MjModel.from_xml_path(xml_path+"_with_object.xml")
else:
    mj_model = mj.MjModel.from_xml_path(xml_path+".xml")

GP = bg.GripperPlatform(mj_model, viewer_enable=True, hardware_enable=hw_mode, log_path=None)

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