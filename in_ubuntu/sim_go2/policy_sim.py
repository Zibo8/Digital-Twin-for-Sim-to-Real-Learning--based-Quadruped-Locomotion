# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
# ------------------------------------------------------------------------------
# Code source: https://github.com/isaac-sim/IsaacSim.git
# Modified by Zibo Xie for educational purposes in 07.2025.
# ------------------------------------------------------------------------------
from typing import Optional
import time
import numpy as np
import omni
import omni.kit.commands
from isaacsim.core.utils.rotations import quat_to_rot_matrix,quat_to_euler_angles
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.storage.native import get_assets_root_path
from tra_spline import CubicSpline
from tra_planner import LinearLocalPlanner
from policy_controller import PolicyController

class Go2FlatTerrainPolicy(PolicyController):
    """The go2 quadruped"""

    def __init__(
        self,
        prim_path: str,
        root_path: Optional[str] = None,
        name: str = "go2",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        """
        Initialize robot and load RL policy.
        Args:
            prim_path (str) -- prim path of the robot on the stage
            root_path (Optional[str]): The path to the articulation root of the robot
            name (str) -- name of the quadruped
            usd_path (str) -- robot usd filepath in the directory
            position (np.ndarray) -- position of the robot
            orientation (np.ndarray) -- orientation of the robot
        """
        assets_root_path = get_assets_root_path()
        if usd_path == None:
            usd_path = assets_root_path + "/Isaac/Robots/Unitree/Go2/go2.usd"
        super().__init__(name, prim_path, root_path, usd_path, position, orientation)
        self.load_policy(
            "/home/cps/in_ubuntu/policy/rsl_flat_noise.pt",
            "/home/cps/in_ubuntu/policy/rsl_flat_noise.yaml"
        )
        # based on env.yaml to set the scale
        self._action_scale = 0.25
        self._previous_action = np.zeros(12)
        self._policy_counter = 0
        # based on real joint positions, we need to set the same iniitial state
        self.real_joint_positions = [0,0,0,0,0.67,0.67,0.67,0.67,-1.3, -1.3, -1.3, -1.3]
        # keep same state as the real robot
        self.count_for_init = 0
        # planner
        self.weight=[3.6,3.6,0]
        self.bezier_following_steps = 2000
        point_array = np.array([[0, 0], [1, 0], [2, 0], [5, 0]], dtype=np.float64) 
        bez_curve = CubicSpline(p0=point_array[0][:], p1=point_array[1][:], p2=point_array[2][:], p3=point_array[3][:])
        spline_list = [bez_curve]
        self.llp=LinearLocalPlanner(spline_list, 1.0)
        # store data
        self.duration = 1222
        self.count = 0
        self.store_pos = []
        self.store_obs = []
        self.store_time = []
        self.store_time_step = []

    def _compute_observation(self, command):
        """
        Compute the observation vector for the policy

        Argument:
        command (np.ndarray) -- the robot command (v_x, v_y, w_z)

        Returns:
        np.ndarray -- The observation vector.

        """
        lin_vel_I = self.robot.get_linear_velocity()
        ang_vel_I = self.robot.get_angular_velocity()
        pos_IB, q_IB = self.robot.get_world_pose()
        R_IB = quat_to_rot_matrix(q_IB)
        R_BI = R_IB.transpose()
        lin_vel_b = np.matmul(R_BI, lin_vel_I)
        ang_vel_b = np.matmul(R_BI, ang_vel_I)
        gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))
        obs = np.zeros(48)
        # linear velocity in body frame
        obs[:3] = lin_vel_b 
        # angular velocity in body frame
        obs[3:6] = ang_vel_b
        obs[6:9] = gravity_b
        """
        you have three choices for the command
        1: use the keyboard to control the robot
            obs[9:12]=command
        2: set the command directly
            obs[9:12]=[1,0,0]
        3: use the planner to send the command
            self.llp.update_position(np.array([pos_IB[0],pos_IB[1])])
            direction = self.llp.get_control_target(np.array([pos_IB[0],pos_IB[1])])
            dir_body = np.matmul(R_BI, np.append(direction, [0]))
            command_vx = np.clip(self.weight[0]*dir_body[0], -1, 1)
            command_vy = np.clip(self.weight[1]*dir_body[1], -1, 1)
            obs[9:12] = [command_vx,command_vy,0]
        """
        obs[9:12]=command
        # Joint states
        current_joint_pos = self.robot.get_joint_positions()
        current_joint_vel = self.robot.get_joint_velocities()
        obs[12:24] = current_joint_pos - self.default_pos
        obs[24:36] = current_joint_vel
        # Previous Action
        obs[36:48] = self._previous_action
        # store data
        if self.count < self.duration - 20 and self.count_for_init==20:
            self.count += 1
            self.store_pos.append(pos_IB)
            self.store_obs.append(obs)
            self.store_time.append(time.time())
            self.store_time_step.append(self.count)
        if self.count == (self.duration -20 -1):
            # np.savetxt("data_pos.txt", np.array(self.store_pos))
            print("Data saved")

        return obs

    def forward(self, dt, command):
        """
        Compute the desired torques and apply them to the articulation
        Argument:
        dt (float) -- Timestep update in the world.
        command (np.ndarray) -- the robot command (v_x, v_y, w_z)
        """
        # warmup, and keep the same initial state with the real robot
        if self.count_for_init < 20:
            self.robot.set_world_pose(position=[0.0, 0.0, 0.385],orientation=[1.0, 0.0, 0.0, 0])
            self.robot.set_joint_positions(self.real_joint_positions)
            self.count_for_init += 1
            print('warmup')
        else:
            obs = self._compute_observation(command)
            if self._policy_counter % self._decimation == 0:
                self.action = self._compute_action(obs)
                self._previous_action = self.action.copy()
            joint_positions=self.default_pos + (self.action * self._action_scale)
            action = ArticulationAction(joint_positions)
            self.robot.apply_action(action)

            self._policy_counter += 1


