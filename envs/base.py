# imports
import brl_gripper as bg
import mujoco as mj
import termios
import sys
import os

class BaseEnv:
    def __init__(self, xml_path='scenev2', gp_type='v2', *args, **kwargs):
        # initialization
        print("Starting init.")
        self.init_settings = termios.tcgetattr(sys.stdin)

        # platform
        xml_path = os.path.join(bg.assets.ASSETS_DIR, xml_path)
 
        hw_mode = bg.HardwareEnable.NO_HW
        mj_model = mj.MjModel.from_xml_path(xml_path+".xml")

        if gp_type == 'v1':
            self.sim = bg.GripperPlatform(mj_model, viewer_enable=True, hardware_enable=hw_mode, log_path=None)
        elif gp_type == 'v2':
            self.sim = bg.GripperPlatformV2(mj_model, viewer_enable=True, hardware_enable=hw_mode, log_path=None)
        