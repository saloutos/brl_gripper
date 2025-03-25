# imports
import brl_gripper as bg
import mujoco as mj
import termios
import sys
import os

import xml.etree.ElementTree as ET
import tempfile

import numpy as np
from envs.utils import get_object_body

class TopDownGraspingEnv:
    def __init__(
            self, 
            xml_path='scenev2', 
            gp_type='v2', 
            obj_cfg={'obj_type': 'box'},
            *args, 
            **kwargs
        ):
        # initialization
        print("Starting init.")
        self.init_settings = termios.tcgetattr(sys.stdin)

        hw_mode = bg.HardwareEnable.NO_HW

        # platform
        xml_path = os.path.join(bg.assets.ASSETS_DIR, xml_path+'.xml')
        
        # Parse the existing XML file
        tree = ET.parse(xml_path)
        root = tree.getroot()
        worldbody = root.find("worldbody")

        # Create a new body for the object (not a geom) with a free joint
        new_body = get_object_body(**obj_cfg)

        # Append the new body to the worldbody element
        if worldbody is not None:
            worldbody.append(new_body)
        else:
            print("Error: <worldbody> element not found in the XML.")

        # Convert the modified XML tree to a string (in-memory)
        xml_string = ET.tostring(root, encoding="unicode")

        # Workaround: Write the XML string to a temporary file
        base_dir = os.path.dirname(xml_path)
        with tempfile.NamedTemporaryFile(suffix=".xml", dir=base_dir, delete=False) as tmp_file:
            tmp_file.write(xml_string.encode("utf-8"))
            tmp_file.flush()
            temp_xml_path = tmp_file.name
            print("Temporary XML file created at:", temp_xml_path)

        # Load the modified model from the temporary XML file
        mj_model = mj.MjModel.from_xml_path(temp_xml_path)
        os.remove(temp_xml_path)

        if gp_type == 'v1': 
            self.sim = bg.GripperPlatform(mj_model, viewer_enable=True, hardware_enable=hw_mode, log_path=None)
        elif gp_type == 'v2':
            self.sim = bg.GripperPlatformV2(mj_model, viewer_enable=True, hardware_enable=hw_mode, log_path=None)
        
        # set initial gripper pose
        body_id = mj.mj_name2id(self.sim.mj_model, mj.mjtObj.mjOBJ_BODY, "floating_2")
        start = self.sim.mj_model.body_dofadr[body_id]
        self.sim.mj_data.qpos[start:start+3] = [0.0, 0.0, 0.5]
        self.sim.mj_data.qpos[start+3:start+7] = [1.0, 0.0, 0.0, 0.0]
        mj.mj_forward(self.sim.mj_model, self.sim.mj_data)
    
    def update(self, time):
        pass