import os
import sys
import time
import mujoco
import mujoco.viewer as viewer
from functools import partial
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '../config'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../linear_mpc'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../utils/'))

from gait import Gait
from leg_controller import LegController
from linear_mpc_configs import LinearMpcConfig
from mpc import ModelPredictiveController
from robot_configs import AliengoConfig
from robot_data import RobotData
from swing_foot_trajectory_generator import SwingFootTrajectoryGenerator

class SimulatorMujoco:
    def __init__(self, asset_path, joint_sensor_names, robot): 
        self.robot = robot
        self.joint_sensor_names = joint_sensor_names
        self.joint_num = len(joint_sensor_names)
        
        # Load the MuJoCo model and data from the specified XML asset path
        self.mujoco_model = mujoco.MjModel.from_xml_path(asset_path)
        self.mujoco_data = mujoco.MjData(self.mujoco_model)
        
        # Launch the MuJoCo viewer in passive mode with custom settings
        self.viewer = viewer.launch_passive(self.mujoco_model, self.mujoco_data, key_callback=self.key_callback, show_left_ui=True, show_right_ui=True)
        self.viewer.cam.distance = 10  # Set camera distance
        self.viewer.cam.elevation = -20  # Set camera elevation
        self.paused = False # Set the simulation to run by default
    
        self.dt = self.mujoco_model.opt.timestep  # Get simulation timestep
        self.fps = 1 / self.dt  # Calculate frames per second (FPS)

        # Initialize robot state data with default values
        self.robot_state = datatypes.RobotState()
        self.robot_state.tau = [0. for x in range(0, self.joint_num)]
        self.robot_state.q = [0. for x in range(0, self.joint_num)]
        self.robot_state.dq = [0. for x in range(0, self.joint_num)]

        # Initialize IMU data structure
        self.imu_data = datatypes.ImuData()

    # Callback for keypress events in the MuJoCo viewer (currently does nothing)
    def key_callback(self, keycode):
        if chr(keycode) == ' ':
            self.paused = not self.paused

    def run(self):
        frame_count = 0
        self.rate = Rate(self.fps)  # Set the update rate according to FPS
        while self.viewer.is_running():   
            if not self.paused: 
                # Step the MuJoCo physics simulation
                mujoco.mj_step(self.mujoco_model, self.mujoco_data)

                # Update robot state data from simulation
                for i in range(self.joint_num):
                    self.robot_state.q[i] = self.mujoco_data.qpos[i + 7]
                    self.robot_state.dq[i] = self.mujoco_data.qvel[i + 6]
                    self.robot_state.tau[i] = self.mujoco_data.ctrl[i]

                    # Apply control commands to the robot based on the received robot command data
                    self.mujoco_data.ctrl[i] = (
                        self.robot_cmd.Kp[i] * (self.robot_cmd.q[i] - self.robot_state.q[i]) + 
                        self.robot_cmd.Kd[i] * (self.robot_cmd.dq[i] - self.robot_state.dq[i]) + 
                        self.robot_cmd.tau[i]
                    )
            
                # Set the timestamp for the current robot state and publish it
                self.robot_state.stamp = time.time_ns()
                self.robot.publishRobotStateForSim(self.robot_state)

                # Extract IMU data (orientation, gyro, and acceleration) from simulation
                imu_quat_id = mujoco.mj_name2id(self.mujoco_model, mujoco.mjtObj.mjOBJ_SENSOR, "quat")
                self.imu_data.quat[0] = self.mujoco_data.sensordata[self.mujoco_model.sensor_adr[imu_quat_id] + 0]
                self.imu_data.quat[1] = self.mujoco_data.sensordata[self.mujoco_model.sensor_adr[imu_quat_id] + 1]
                self.imu_data.quat[2] = self.mujoco_data.sensordata[self.mujoco_model.sensor_adr[imu_quat_id] + 2]
                self.imu_data.quat[3] = self.mujoco_data.sensordata[self.mujoco_model.sensor_adr[imu_quat_id] + 3]

                imu_gyro_id = mujoco.mj_name2id(self.mujoco_model, mujoco.mjtObj.mjOBJ_SENSOR, "gyro")
                self.imu_data.gyro[0] = self.mujoco_data.sensordata[self.mujoco_model.sensor_adr[imu_gyro_id] + 0]
                self.imu_data.gyro[1] = self.mujoco_data.sensordata[self.mujoco_model.sensor_adr[imu_gyro_id] + 1]
                self.imu_data.gyro[2] = self.mujoco_data.sensordata[self.mujoco_model.sensor_adr[imu_gyro_id] + 2]

                imu_acc_id = mujoco.mj_name2id(self.mujoco_model, mujoco.mjtObj.mjOBJ_SENSOR, "acc")
                self.imu_data.acc[0] = self.mujoco_data.sensordata[self.mujoco_model.sensor_adr[imu_acc_id] + 0]
                self.imu_data.acc[1] = self.mujoco_data.sensordata[self.mujoco_model.sensor_adr[imu_acc_id] + 1]
                self.imu_data.acc[2] = self.mujoco_data.sensordata[self.mujoco_model.sensor_adr[imu_acc_id] + 2]

                # Set the timestamp for the current IMU data and publish it
                self.imu_data.stamp = time.time_ns()
                self.robot.publishImuDataForSim(self.imu_data)

            # Sync the viewer every 20 frames for smoother visualization
            if frame_count % 20 == 0:
                self.viewer.sync()

            frame_count += 1
            self.rate.sleep()  # Maintain the simulation loop at the correct rate

if __name__ == '__main__': 


    parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    print(f"*** Parent Directory: {parent_dir} ***")
    
    # Define the path to the robot model XML file based on the robot type
    model_path = f"{parent_dir}/robot/aliengo/robot.xml"

    # Check if the model file exists, otherwise exit with an error
    if not os.path.exists(model_path):
        print(f"Error: The file {model_path} does not exist. Please ensure the ROBOT_TYPE is set correctly.")
        sys.exit(1)


    # Create and run the MuJoCo simulator instance
    simulator = SimulatorMujoco(model_path, robot)
    simulator.run()
