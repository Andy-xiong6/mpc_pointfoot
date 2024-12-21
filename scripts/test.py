import os
import sys
import time
import mujoco
import mujoco.viewer as viewer
from functools import partial
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '../config'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../convex_mpc'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../utils/'))

from gait import Gait
from leg_controller import LegController
from linear_mpc_configs import LinearMpcConfig
from mpc import ModelPredictiveController
from robot_configs import AliengoConfig
from robot_data import RobotData
from swing_foot_trajectory_generator import SwingFootTrajectoryGenerator

parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Define the path to the robot model XML file based on the robot type
model_path = f"{parent_dir}/robot/aliengo/robot.xml"


mujoco_model = mujoco.MjModel.from_xml_path(model_path)
mujoco_data = mujoco.MjData(mujoco_model)
viewer = viewer.launch_passive(mujoco_model, mujoco_data, show_left_ui=True, show_right_ui=True)
while viewer.is_running():
    mujoco.mj_step(mujoco_model, mujoco_data)
print(mujoco_model.joint)
print(mujoco_data.qpos)

print(f"*** MuJoCo Model: {mujoco_model} ***")