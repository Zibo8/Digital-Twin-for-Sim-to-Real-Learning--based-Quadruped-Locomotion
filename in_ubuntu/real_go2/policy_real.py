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
import io
from typing import Optional
import numpy as np
import omni
import torch
from config_loader import  get_physics_properties, parse_env_config

class PolicyReal():
    def __init__(self):
        # based on env.yaml to set the action scale
        self._action_scale = 0.25
        self.previous_action = np.zeros(12)
        self._policy_counter = 0
        self.load_policy(
            "/home/cps/in_ubuntu/policy/rsl_flat_noise.pt",
            "/home/cps/in_ubuntu/policy/rsl_flat_noise.yaml"
        )
        print("load_policy")

    def load_policy(self, policy_file_path, policy_env_path) -> None:
        """
        Loads a policy from a file.

        Args:
            policy_file_path (str): The path to the policy file.
            policy_env_path (str): The path to the environment configuration file.
        """
        file_content = omni.client.read_file(policy_file_path)[2]
        file = io.BytesIO(memoryview(file_content).tobytes())
        self.policy = torch.jit.load(file)
        self.policy_env_params = parse_env_config(policy_env_path)

        self._decimation, self._dt, self.render_interval = get_physics_properties(self.policy_env_params)

    def _compute_action(self, obs: np.ndarray) -> np.ndarray:
        """
        Computes the action from the observation using the loaded policy.

        Args:
            obs (np.ndarray): The observation.

        Returns:
            np.ndarray: The action.
        """
        with torch.no_grad():
            obs = torch.from_numpy(obs).view(1, -1).float()
            action = self.policy(obs).detach().view(-1).numpy()
            # print("action:", action)
        return action
    
    def forward(self, dt, obs,default_pos):
        """
        Compute the desired torques and apply them to the articulation

        Argument:
        dt (float) -- Timestep update in the world.
        command (np.ndarray) -- the robot command (v_x, v_y, w_z)

        """
        self.action = self._compute_action(obs)
        joint_positions=default_pos + (self.action * self._action_scale)
        return joint_positions,self.action

