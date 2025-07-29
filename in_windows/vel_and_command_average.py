# Code originally provided by Hongpengcao.
# DataStream SDK is from vicon company
# ------------------------------------------------------------------------------
# Modified by Zibo Xie for educational purposes in 07.2025.
# ------------------------------------------------------------------------------
from __future__ import print_function
import argparse
import time
import pickle
import redis
from vicon_dssdk import ViconDataStream
import copy
from tra_spline import CubicSpline
from tra_planner import LinearLocalPlanner
import numpy as np
from collections import deque

# ---------------------------
# Argument parsing
# ---------------------------
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    'host',
    nargs='?',
    help="Vicon Server address in the format server:port",
    default="localhost:801"
)
args = parser.parse_args()

# ---------------------------
# Redis connection setup
# ---------------------------
pool = redis.ConnectionPool(host="192.168.123.51", port=6379, password='ubuntu')
redis_cli = redis.Redis(connection_pool=pool)
REDIS_CHANNEL = 'channel_pose'

def publish_states(states):
    """Package and publish the states to Redis"""
    payload = pickle.dumps(states)
    redis_cli.publish(REDIS_CHANNEL, payload)

# ---------------------------
# Initialize Vicon DataStream client
# ---------------------------
client = ViconDataStream.Client()

# 1) Retry connection until successful
print(f"Connecting to Vicon at {args.host} ...")
while True:
    try:
        client.Connect(args.host)
        print("✔ Connected")
        break
    except ViconDataStream.DataStreamException as e:
        print(f"✖ Connect failed, retrying in 1s: {e}")
        time.sleep(1.0)

# 2) Configure client settings
client.SetBufferSize(1)
client.EnableSegmentData()
client.SetStreamMode(ViconDataStream.Client.StreamMode.EServerPush)
client.SetAxisMapping(
    ViconDataStream.Client.AxisMapping.EForward,
    ViconDataStream.Client.AxisMapping.ELeft,
    ViconDataStream.Client.AxisMapping.EUp
)

print("Configuration completed!")

# Discard the first few frames to ensure data is available
for _ in range(5):
    try:
        client.GetFrame()
    except ViconDataStream.DataStreamException:
        pass

fre = client.GetFrameRate()
dt = 1/fre
print("Frame Rate:", client.GetFrameRate(), "Hz")
print("Total Latency:", client.GetLatencyTotal(), "ms")

# warmup and init_position
init_position = [0,0,0]
for _ in range(5):
    try:
        client.GetFrame()
        for subject in client.GetSubjectNames():
            for segment in client.GetSegmentNames(subject):
                # init_position, unit is meter
                init_position[0] = (client.GetSegmentGlobalTranslation(subject, segment)[0][0]) / 1000
                init_position[1] = (client.GetSegmentGlobalTranslation(subject, segment)[0][1]) / 1000
                init_position[2] = (client.GetSegmentGlobalTranslation(subject, segment)[0][2]) / 1000
    except ViconDataStream.DataStreamException:
        pass
print("init_pos",init_position)

# ---------------------------
# init for calculating velocity
# ---------------------------
vel = np.zeros(3)
length_win = 3
vx_window=deque([0,0,0],maxlen=length_win)
vy_window=deque([0,0,0],maxlen=length_win)
vz_window=deque([0,0,0],maxlen=length_win)
prev_pos = copy.deepcopy(np.array(init_position))
limit_vel = 3

# ---------------------------
# filter for postions
# ---------------------------
pos_x_window=deque([0,0,0],maxlen=length_win)
pos_y_window=deque([0,0,0],maxlen=length_win)
pos_z_window=deque([0,0,0],maxlen=length_win)
limit_pos_x = 2.5 # meter
limit_pos_y = 2.5 # meter
limit_pos_z = 0.6 # meter
pos_offset_z = -0.086

command = np.zeros(3)
# ++++++++++++++++

weight_com = 2.2
# ++++++++++++++++

""" store data"""
store_err = []
store_vel = []
store_time = []
# ---------------------------
# Main loop: fetch frames & publish
# ---------------------------
counter = 0
while True:
    try:
        # Block until a new frame arrives (approximately 100 Hz)
        client.GetFrame()
        data_all = []
        for subject in client.GetSubjectNames():
            for segment in client.GetSegmentNames(subject):
                # Retrieve position and quaternion data
                translation = client.GetSegmentGlobalTranslation(subject, segment)
                quaternion  = client.GetSegmentGlobalRotationQuaternion(subject, segment)
                counter += 1
                
                """velocity """
                pos=copy.deepcopy(np.array(translation[0])/1000)
                raw_vel = (pos-prev_pos)/dt
                # only 3m/s are considered as correct value
                if abs(raw_vel[0]) < limit_vel:
                    vx_window.append(raw_vel[0])
                if abs(raw_vel[1]) < limit_vel:
                    vy_window.append(raw_vel[1])
                if abs(raw_vel[2]) < limit_vel:
                    vz_window.append(raw_vel[2])  
                vel = [sum(vx_window)/length_win,sum(vy_window)/length_win,sum(vz_window)/length_win]
                prev_pos=copy.deepcopy(np.array(translation[0])/1000)
           
                """motion planning"""
                final_com = [float(0),float(0),0]

                """filter for positions"""
                if abs(pos[0]) < limit_pos_x:
                    pos_x_window.append(pos[0])
                if abs(pos[1]) < limit_pos_y:
                    pos_y_window.append(pos[1])
                if abs(pos[2]) < limit_pos_z:
                    pos_z_window.append(pos[2])
                pos_send_array_ele = [sum(pos_x_window)/length_win,sum(pos_y_window)/length_win,sum(pos_z_window)/length_win+pos_offset_z]
                """" type of sending: no numpy, only list  """
                vel_send = [float(x) for x in vel]
                pos_send = [float(x) for x in pos_send_array_ele]
                data_all.append({
                    'subjectname':  subject,
                    'segmentName':  segment,
                    'translation':  translation,
                    'quaternion':  quaternion,
                    'ind':          counter,
                    'position':     pos_send,
                    'vel':          vel_send,
                    'command':      final_com,
                })

        #         # store data and save
        #         store_vel.append(vel_send)
        #         store_time.append(time.time())
        # if counter == 3000:
        #     np.savetxt("average_vel.txt",np.array(store_vel))
        #     np.savetxt("average_time.txt",np.array(store_time))
        #     print("save done")

        publish_states(data_all)
        # print(quaternion)
        # print('err',error)
        # print("pos",pos_send)
        # print("raw_vel",raw_vel)
        # print("mea_vel",vel_send)
        # print('command',final_com)


    except ViconDataStream.DataStreamException as e:
        # Handle intermittent data stream errors and continue fetching
        print("Data stream error (recovering):", e)
        # Do not sleep; let GetFrame() block and retry immediately
        continue
