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
from isaacsim import SimulationApp
simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

import omni
import time
import sys
import carb
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.storage.native import get_assets_root_path
import numpy as np
# my codes
from redis_vel import RedisVel
from redis_digital_twin import RedisDigitalTwin

class Subscriber():
    def __init__(self):
        # setting up the world
        self.timeline = omni.timeline.get_timeline_interface()
        self.ros_world = World(stage_units_in_meters=1.0)
        self.ros_world.scene.add_default_ground_plane() # add ground plane
        physics_context = self.ros_world.get_physics_context()
        physics_context.set_gravity(0.0)# set gravity to 0
        # load go2
        self.assets_root_path = get_assets_root_path()
        self.asset_path = self.assets_root_path + "/Isaac/Robots/Unitree/Go2/go2.usd"
        self.prim=add_reference_to_stage(usd_path=self.asset_path, prim_path="/World/Go2")  # add robot to stage
        self.go2 = Articulation(prim_paths_expr="/World/Go2", name="my_Go2")  # create an articulation object
        # Initialize positions, orientations, and joint positions
        self._position = np.array([[0, 0, 0.3]])
        self._orientations = np.array([[0.0, 0.0, 0.0, 1.0]])
        self.go2.set_world_poses(positions=self._position, orientations=self._orientations)
        # redis
        self.redis_vel = RedisVel()
        self.redis_digital_twin = RedisDigitalTwin()
        self.ros_world.reset()

    def run_simulation(self):
        self.timeline.play()
        reset_needed = False
        while simulation_app.is_running():
            self.ros_world.step(render=True)
            if self.ros_world.is_stopped() and not reset_needed:
                reset_needed = True
            if self.ros_world.is_playing():
                if reset_needed:
                    self.ros_world.reset()
                    reset_needed = False 
                self.go2.set_world_poses(positions=np.array([[self.redis_vel.pos]]), orientations=np.array([[self.redis_digital_twin.quaternion]]))
                self.go2.set_joint_positions(self.redis_digital_twin.joint_pos)
                time.sleep(0.02) #50hz

        # Cleanup
        self.timeline.stop()
        simulation_app.close()


if __name__ == "__main__":
    subscriber = Subscriber()
    subscriber.run_simulation()
