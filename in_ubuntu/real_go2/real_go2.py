# ------------------------------------------------------------------------------
# Code source: https://github.com/unitreerobotics/unitree_sdk2_python.git
# Modified by Zibo Xie for educational purposes in 07.2025.
# ------------------------------------------------------------------------------
# isaacsim
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})
import io
import omni
# unitree sdk2py
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
import unitree_legged_const as go2
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient
# standard library
import time
import sys
import numpy as np
import copy
import redis
import pickle
from typing import Optional
import torch
# mycode
from policy_real import PolicyReal
from redis_vel import RedisVel
from quat_to_rot_matrix import quat_to_rot_matrix
from change_joint_order import unitree_joint_order_to_isaac, isaac_joint_order_to_unitree

class Custom:
    def __init__(self):
        self.Kp = 60.0
        self.Kd = 5
        self.time_consume = 0
        self.rate_count = 0
        self.sin_count = 0
        self.motiontime = 0
        self.dt = 0.005  # 0.001~0.01

        self.low_cmd = unitree_go_msg_dds__LowCmd_()  
        self.low_state = None  

        self._targetPos_1 = [0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                             -0.2, 1.36, -2.65, 0.2, 1.36, -2.65]
        self._targetPos_2 = [0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                             0.0, 0.67, -1.3, 0.0, 0.67, -1.3]
        self._targetPos_3 = [-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                             -0.5, 1.36, -2.65, 0.5, 1.36, -2.65]

        self.startPos = [0.0] * 12
        self.duration_1 = 500
        self.duration_2 = 1222
        self.duration_3 = 500
        self.duration_4 = 500
        self.percent_1 = 0
        self.percent_2 = 0
        self.percent_3 = 0
        self.percent_4 = 0

        self.firstRun = True
        self.done = False

        # thread handling
        self.lowCmdWriteThreadPtr = None

        self.crc = CRC()

        # class: policy
        self.poilcy = PolicyReal()
        # class: redis
        self.redis=RedisVel()

        #obs
        self.obs=np.zeros(48)
        self.vel=np.zeros(3)
        self.rot_matrix = np.eye(3)
        self.previous_action = np.zeros(12)
        self.joint_pos_command_unitree = self._targetPos_2
        self.unitree_joint_pos = np.zeros(12)
        self.unitree_joint_vel = np.zeros(12)
        self.grav=np.zeros(3)
        self.counter=0
        # the robot command (v_x, v_y, w_z)
        self.command = np.zeros(3)
        # based on env.yaml
        self.default_pos=[0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5]
        # warmup
        for _ in range(10):
           _, _ = self.poilcy.forward(self.dt, self.obs, self.default_pos)
        print("Initialization done")
        # store data
        self.store_time = []
        self.store_time_step = []
        self.store_quat_vicon = []
        self.store_pos = []
        self.store_obs = []
        self.flag_policy = False
        self.keep_standing_count = 0

        # digital twin
        self.pool = redis.ConnectionPool(host="192.168.123.51", port=6379, password='ubuntu')
        self.redis_cli = redis.Redis(connection_pool=self.pool)
        self.REDIS_CHANNEL = 'channel_digital_twin'
        self.data_digital_twin = []

    def publish_states(self,states):
        payload = pickle.dumps(states)
        self.redis_cli.publish(self.REDIS_CHANNEL, payload)

    def Init(self):
        self.InitLowCmd()

        # create publisher #
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()

        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)

        self.sc = SportClient()  
        self.sc.SetTimeout(5.0)
        self.sc.Init()

        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()
        while result['name']:
            self.sc.StandDown()
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=0.005, target=self.LowCmdWrite, name="writebasiccmd"
        )
        self.lowCmdWriteThreadPtr.Start()

    def InitLowCmd(self):
        self.low_cmd.head[0]=0xFE
        self.low_cmd.head[1]=0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(20):
            self.low_cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            self.low_cmd.motor_cmd[i].q= go2.PosStopF
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = go2.VelStopF
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = 0

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg
        self.vel[0]=self.redis.vel[0] 
        self.vel[1]=self.redis.vel[1] 
        self.vel[2]= self.redis.vel[2]
        # transform
        quat= msg.imu_state.quaternion
        self.rot_matrix = quat_to_rot_matrix(quat)
        vel_body = np.matmul(self.rot_matrix.transpose(),self.vel)
        gravity_b = np.matmul(self.rot_matrix.transpose(), np.array([0.0, 0.0, -1.0]))
        self.obs[0:3]=vel_body
        self.obs[3:6] = msg.imu_state.gyroscope
        self.obs[6:9] = gravity_b
        # 1: constant command: self.obs[9:12] = [0.6,0,0]
        # 2: path planner command: self.obs[9:12] = self.redis.command
        if self.counter < (self.duration_2-20):
            self.obs[9:12] = [0.6,0,0]
        else:
            self.obs[9:12] = [0,0,0]
        for i in range(12):
            self.unitree_joint_pos[i] = msg.motor_state[i].q
            self.unitree_joint_vel[i] = msg.motor_state[i].dq
        isaac_joint_pos = unitree_joint_order_to_isaac(self.unitree_joint_pos)
        isaac_joint_vel = unitree_joint_order_to_isaac(self.unitree_joint_vel)
        self.obs[12:24] = isaac_joint_pos - self.default_pos
        self.obs[24:36] = isaac_joint_vel
        self.obs[36:48] = self.previous_action
        # store data
        if self.keep_standing_count > 10 and self.keep_standing_count < 50:
            self.store_quat_vicon.append(self.redis.quat_vicon)
        if self.counter < (self.duration_2-20) and self.flag_policy:
            self.store_pos.append(self.redis.raw_pos)
            obs_list = self.obs.flatten().tolist()
            self.store_obs.append(obs_list)
            self.store_time.append(time.time())
            self.store_time_step.append(self.counter)
            self.flag_policy = False
        if self.counter == (self.duration_2-20) and self.flag_policy:
            # np.savetxt('data_pos_real.txt',np.array(self.store_pos))
            print('txt save')
            self.flag_policy = False
        # digital twin
        self.data_digital_twin.append({
            'joint_positions': isaac_joint_pos,
            'quaternion':quat,
        })
        self.publish_states(self.data_digital_twin)
        self.data_digital_twin= []

    def LowCmdWrite(self):
        if self.firstRun:
            for i in range(12):
                self.startPos[i] = self.low_state.motor_state[i].q
            self.firstRun = False

        self.percent_1 += 1.0 / self.duration_1
        self.percent_1 = min(self.percent_1, 1.2)
        if self.percent_1 < 1:
            for i in range(12):
                self.low_cmd.motor_cmd[i].q = (1 - self.percent_1) * self.startPos[i] + self.percent_1 * self._targetPos_2[i]
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd
                self.low_cmd.motor_cmd[i].tau = 0
        elif self.percent_1 < 1.2:
            self.keep_standing_count += 1
            for i in range(12):
                self.low_cmd.motor_cmd[i].q = self._targetPos_2[i] 
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd
                self.low_cmd.motor_cmd[i].tau = 0

        if (self.percent_1 == 1.2) and (self.percent_2 < 1):
            self.percent_2 += 1.0 / self.duration_2
            self.percent_2 = min(self.percent_2, 1)
            # policy
            if  self.counter % 4==0:    
                # this step costs < 1 ms on pc
                joint_pos_command_issac, self.previous_action = self.poilcy.forward(self.dt, self.obs, self.default_pos)
                self.joint_pos_command_unitree = isaac_joint_order_to_unitree(joint_pos_command_issac)
            self.counter += 1
            # flag for storing data
            self.flag_policy = True
            self.Kp = 25
            self.Kd = 0.5
            for i in range(12):
                self.low_cmd.motor_cmd[i].q = self.joint_pos_command_unitree[i]
                # self.low_cmd.motor_cmd[i].q = self._targetPos_2[i] 
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd
                self.low_cmd.motor_cmd[i].tau = 0

        if (self.percent_1 == 1.2) and (self.percent_2 == 1) and (self.percent_3 < 1):
            self.percent_3 += 1.0 / self.duration_3
            self.percent_3 = min(self.percent_3, 1)
            self.Kp = 60
            self.Kd = 5
            for i in range(12):
                self.low_cmd.motor_cmd[i].q = self._targetPos_2[i]
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd
                self.low_cmd.motor_cmd[i].tau = 0

        if (self.percent_1 == 1.2) and (self.percent_2 == 1) and (self.percent_3 == 1) and (self.percent_4 <= 1):
            self.percent_4 += 1.0 / self.duration_4
            self.percent_4 = min(self.percent_4, 1)
            for i in range(12):
                self.low_cmd.motor_cmd[i].q = (1 - self.percent_4) * self._targetPos_2[i] + self.percent_4 * self._targetPos_3[i]
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd
                self.low_cmd.motor_cmd[i].tau = 0

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)


if __name__ == '__main__':

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    custom = Custom()
    custom.Init()
    custom.Start()

    while True:        
        if custom.percent_4 == 1.0: 
           time.sleep(1)
           print("Done!")
           sys.exit(-1)     
        time.sleep(1)