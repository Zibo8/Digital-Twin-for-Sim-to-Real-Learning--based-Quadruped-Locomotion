import numpy as np
import matplotlib.pyplot as plt
from tra_spline import CubicSpline
import math
"""
different axis !!!
vicon: forward: x
plot : forward: y
"""
file_dict = {
    './txt_file/for_pos.txt': {
                    'pos_trial_1':"./txt_file/for_pos_trial1.txt",
                    'pos_trial_2':"./txt_file/for_pos_trial2.txt",
                    'pos_trial_3':"./txt_file/for_pos_trial3.txt",
                    'quat_trial_1':"./txt_file/for_quat_vicon_trial1.txt",
                    'quat_trial_2':"./txt_file/for_quat_vicon_trial2.txt",
                    'quat_trial_3':"./txt_file/for_quat_vicon_trial3.txt",
                    'pos_trial_1':"./txt_file/for_pos_trial1.txt",
                    'pos_trial_2':"./txt_file/for_pos_trial2.txt",
                    'pos_trial_3':"./txt_file/for_pos_trial3.txt",
                    'time_file':'./txt_file/for_time.txt',
                    'obs_file':'./txt_file/for_obs.txt',
                    'sim_file':'./txt_file/sim_for_pos.txt',
                    'sim_time_file':'./txt_file/sim_for_time.txt',
                    'sim_obs_file':'./txt_file/sim_for_obs.txt',
                    'label': 'forward',
                    'curve': [0, 1, 2, 3, 0, 0, 0, 0], #[x1,x2,x3,x4,y1,y2,y3,y4] 
                    'color':'green',
                    '00_range_x': [-2, 3],
                    '00_range_y': [-0.5, 3.5],
                    '01_range_y': [-0.5, 3],
                    '10_range_y': [-1, 1],
                    '11_range_y': [0.25, 0.55],
    },
    # './txt_file/back_pos.txt': {
    #                 'time_file':'./txt_file/back_time.txt',
    #                 'obs_file':'./txt_file/back_obs.txt',
    #                 'sim_file':'./txt_file/sim_back_pos.txt',
    #                 'sim_time_file':'./txt_file/sim_back__time.txt',
    #                 'sim_obs_file':'./txt_file/sim_back_obs.txt',
    #                 'label': 'back',
    #                 'curve': [0, 1, 2, 3, 0, 0, 0, 0], #[x1,x2,x3,x4,y1,y2,y3,y4] 
    #                 'color':'green',
    #                 '00_range_x': [-2, 3],
    #                 '00_range_y': [-3.5, 0.5],
    #                 '01_range_y': [-3, 0.5],
    #                 '10_range_y': [-1, 1],
    #                 '11_range_y': [0.25, 0.55],
    # },
}

for file_i, config in file_dict.items():
    # load file
    data = np.loadtxt(file_i) 
    data_obs = np.loadtxt(config['obs_file'])
    data_sim_obs = np.loadtxt(config['sim_obs_file'])
    data_sim = np.loadtxt(config['sim_file'])
    data_time = np.arange(1, len(data)+1, dtype=int)
    data_sim_time = np.arange(1, len(data_sim)+1, dtype=int)
    print('load file')
  
    # get real data 
    data_x = np.zeros(len(data))
    data_y = np.zeros(len(data))
    data_z = np.zeros(len(data))
    data_obs_command = np.zeros((len(data),3))
    for i in range(len(data)):
        data_x[i] = -data[i,1] # different axis
        data_y[i] = data[i,0] # different axis
        data_z[i] = data[i,2] # different axis
        data_obs_command[i,:]=data_obs[i,9:12]

    # get sim data
    data_sim_x = np.zeros(len(data_sim))
    data_sim_y = np.zeros(len(data_sim))
    data_sim_z = np.zeros(len(data_sim))
    data_sim_obs_command = np.zeros((len(data),3))
    for i in range(len(data_sim)):
        data_sim_x[i] = -data_sim[i,1] # different axis
        data_sim_y[i] = data_sim[i,0] # different axis
        data_sim_z[i] = data_sim[i,2] # different axis
        data_sim_obs_command[i,:]=data_sim_obs[i,9:12]

    # offset: vicon
    offset_z = data_sim_z[0] - data_z[0]
    
     # init real data
    data_x = data_x - data_x[0]
    data_y = data_y - data_y[0]
    data_z = data_z + offset_z
    
    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # plot-trajectory
    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    fig, axs = plt.subplots(2, 2, figsize=(10, 8)) 
   
    axs[0,0].scatter(data_x, data_y, color='green', label='real trajectory',marker='o',s=20)
    axs[0,0].scatter(data_sim_x, data_sim_y, color='blue', label='sim trajectory',s=20)
    axs[0, 0].set_xlim(config['00_range_x'])
    axs[0, 0].set_ylim(config['00_range_y'])
    axs[0, 0].set_xlabel("x_direction(m)")
    axs[0, 0].set_ylabel("y_direction(m)")
    axs[0, 0].set_title(f"{config['label']}: trajectory")
    axs[0,0].scatter(data_x[0], data_y[0], color='red', label='start point',marker='x', s=35)
    axs[0,0].scatter(data_x[-1], data_y[-1], color='red', label='end point',marker='^', s=35)
    axs[0,0].scatter(data_sim_x[-1], data_sim_y[-1], color='red',marker='^', s=35)

    axs[1, 0].scatter(data_time, data_x, color='green', label='real data')
    axs[1, 0].scatter(data_sim_time, data_sim_x, color='blue', label='sim data')
    axs[1, 0].set_ylim(config['10_range_y'])
    axs[1, 0].set_xlabel("Time step")
    axs[1, 0].set_ylabel("x_direction (m)")
    axs[1, 0].set_title("x_direction")
    axs[1, 0].scatter(data_time[0], data_x[0], color='red', label='start point',marker='x', s=35)
    axs[1, 0].scatter(data_time[-1], data_x[-1], color='red', label='end point',marker='^', s=35)
    axs[1, 0].scatter(data_sim_time[-1], data_sim_x[-1], color='red',marker='^', s=35)

    axs[0, 1].scatter(data_time, data_y, color='green', label='real data')
    axs[0, 1].scatter(data_sim_time, data_sim_y, color='blue', label='sim data')
    axs[0, 1].set_ylim(config['01_range_y'])
    axs[0, 1].set_xlabel("Time step")
    axs[0, 1].set_ylabel("y_direction (m)")
    axs[0, 1].set_title(f"y_direction")
    axs[0, 1].scatter(data_time[0], data_y[0], color='red', label='start point',marker='x', s=35)
    axs[0, 1].scatter(data_time[-1], data_y[-1], color='red', label='end point',marker='^', s=35)
    axs[0, 1].scatter(data_sim_time[-1], data_sim_y[-1], color='red',marker='^', s=35)

    axs[1, 1].scatter(data_time, data_z, color='green', label='real data')
    axs[1, 1].scatter(data_sim_time, data_sim_z, color='blue', label='sim data')
    axs[1, 1].set_ylim(config['11_range_y'])
    axs[1, 1].set_xlabel("Time step")
    axs[1, 1].set_ylabel("z_direction (m)")
    axs[1, 1].set_title(f"z_direction")
    axs[1, 1].scatter(data_time[0], data_z[0], color='red', label='start point',marker='x', s=35)
    axs[1, 1].scatter(data_time[-1], data_z[-1], color='red', label='end point',marker='^', s=35)
    axs[1, 1].scatter(data_sim_time[-1], data_sim_z[-1], color='red',marker='^', s=35)

    # share configration and legend
    for ax in axs.flatten():
        ax.legend()
        ax.grid(True)
    plt.subplots_adjust(hspace=0.3)

    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # plot-obs
    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    fig, axs = plt.subplots(2, 2, figsize=(10, 8)) 
   
    axs[0, 1].scatter(data_time,data_obs_command[:,0],color='green', label='real',marker='o',s=20)
    axs[0, 1].scatter(data_time, data_sim_obs_command[:,0], color='blue', label='sim',s=20)
    axs[0, 1].set_xlabel("time")
    axs[0, 1].set_ylabel("command_y")
    axs[0, 1].set_title(f"{config['label']}:command-y")

    axs[1, 0].scatter(data_time,data_obs_command[:,1],color='green', label='real',marker='o',s=20)
    axs[1, 0].scatter(data_time, data_sim_obs_command[:,1], color='blue', label='sim',s=20)
    axs[1, 0].set_xlabel("time")
    axs[1, 0].set_ylabel("command_x")
    axs[1, 0].set_title(f"{config['label']}:command-x")

    axs[1, 1].scatter(data_time,data_obs_command[:,2],color='green', label='real',marker='o',s=20)
    axs[1, 1].scatter(data_time, data_sim_obs_command[:,2], color='blue', label='sim',s=20)
    axs[1, 1].set_xlabel("time")
    axs[1, 1].set_ylabel("command_z")
    axs[1, 1].set_title(f"{config['label']}:command-yaw")

    for ax in axs.flatten():
        ax.legend()
        ax.grid(True)
    plt.subplots_adjust(hspace=0.3)

    """plot bezier curve"""
    # point_array = np.array([[data_y[0]+curve_dict[file_i][4],data_x[0]+curve_dict[file_i][0]], 
    #                         [data_y[0]+curve_dict[file_i][5],data_x[0]+curve_dict[file_i][1]], 
    #                         [data_y[0]+curve_dict[file_i][6],data_x[0]+curve_dict[file_i][2]], 
    #                         [data_y[0]+curve_dict[file_i][7],data_x[0]+curve_dict[file_i][3]]],dtype=np.float64)
    # bez_curve = CubicSpline(
    # p0=point_array[0], p1=point_array[1],
    # p2=point_array[2], p3=point_array[3])
    # us = np.linspace(0, 1, 200)
    # points = np.array([bez_curve.get_position(u) for u in us])
    # # curve_x = points[:, 0]
    # plt.plot(points[:, 0], points[:, 1],linewidth = 2)

plt.show()

