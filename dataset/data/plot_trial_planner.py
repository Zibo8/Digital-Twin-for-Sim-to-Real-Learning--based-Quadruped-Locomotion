import numpy as np
import matplotlib.pyplot as plt
import math
"""
vicon and isaacsim的坐标轴一样
没有转换axis
"""
file_dict = {
    'forward_high': {
                    'pos_sim':"./txt_file/for_pos_high_sim.txt",
                    'pos_trial_1':"./txt_file/for_pos_high_trial_1.txt",
                    'pos_trial_2':"./txt_file/for_pos_high_trial_2.txt",
                    'pos_trial_3':"./txt_file/for_pos_high_trial_3.txt",
                    'quat_trial_1':"./txt_file/for_quat_vicon_high_trial_1.txt",
                    'quat_trial_2':"./txt_file/for_quat_vicon_high_trial_2.txt",
                    'quat_trial_3':"./txt_file/for_quat_vicon_high_trial_3.txt",
                    'time_step_sim':"./txt_file/for_time_step_high_sim.txt",
                    'time_step_trial_1':"./txt_file/for_time_step_high_trial_1.txt",
                    'time_step_trial_2':"./txt_file/for_time_step_high_trial_2.txt",
                    'time_step_trial_3':"./txt_file/for_time_step_high_trial_3.txt",
                    'label': 'forward with high speed',
                    'color':'green',
                    '00_range_x': [-0.5, 3.5],
                    '00_range_y': [-1, 1],
                    '01_range_y': [-0.5, 0.5],
                    '10_range_y': [-1, 1],
                    '11_range_y': [0.25, 0.55],
    },
}

def quat_to_rot_matrix(q):
    x, y, z, w = q
    R = np.array([
        [1 - 2*(y**2 + z**2),     2*(x*y - z*w),       2*(x*z + y*w)],
        [    2*(x*y + z*w),   1 - 2*(x**2 + z**2),     2*(y*z - x*w)],
        [    2*(x*z - y*w),       2*(y*z + x*w),   1 - 2*(x**2 + y**2)]
    ])
    return R

def average_quaternions(quats_xyzw):
    M = np.zeros((4, 4))
    for q in quats_xyzw:
        v = q.reshape(4, 1)
        M += v @ v.T    
    vals, vecs = np.linalg.eig(M)
    q_avg = vecs[:, np.argmax(vals)]
    if q_avg[3] < 0:
        q_avg = -q_avg
    return q_avg / np.linalg.norm(q_avg)

for file_i, config in file_dict.items():
    # load file
    pos_trial_1 = np.loadtxt(config['pos_trial_1'])
    pos_trial_2 = np.loadtxt(config['pos_trial_2'])
    pos_trial_3 = np.loadtxt(config['pos_trial_3'])
    pos_sim = np.loadtxt(config['pos_sim'])
    quat_trial_1 = np.loadtxt(config['quat_trial_1'])
    quat_trial_2 = np.loadtxt(config['quat_trial_2'])
    quat_trial_3 = np.loadtxt(config['quat_trial_3'])
    time_step_1 = np.loadtxt(config['time_step_trial_1'])
    time_step_2 = np.loadtxt(config['time_step_trial_2'])
    time_step_3 = np.loadtxt(config['time_step_trial_3'])
    time_step_sim = np.loadtxt(config['time_step_sim'])
    print('load file')
    
    # initialize data
    pos_trial_1 = pos_trial_1/1000  # convert mm to m
    pos_trial_2 = pos_trial_2/1000  # convert mm to m
    pos_trial_3 = pos_trial_3/1000  # convert mm to m
    pos_trial_1 = pos_trial_1 - pos_trial_1[0,:]
    pos_trial_2 = pos_trial_2 - pos_trial_2[0,:]
    pos_trial_3 = pos_trial_3 - pos_trial_3[0,:]
    pos_sim = pos_sim - pos_sim[0,:]

    # transform based on quaternion
    quat_average_1 = average_quaternions(quat_trial_1)
    quat_average_2 = average_quaternions(quat_trial_2)
    quat_average_3 = average_quaternions(quat_trial_3)
    rot_matrix_trail_1 = quat_to_rot_matrix(quat_average_1)
    rot_matrix_trail_2 = quat_to_rot_matrix(quat_average_2)
    rot_matrix_trail_3 = quat_to_rot_matrix(quat_average_3)
    for i in range(len(pos_trial_1)):
        pos_trial_1[i,:] = rot_matrix_trail_1.T @ pos_trial_1[i,:]
    for i in range(len(pos_trial_2)):
        pos_trial_2[i,:] = rot_matrix_trail_2.T @ pos_trial_2[i,:]
    for i in range(len(pos_trial_3)):
        pos_trial_3[i,:] = rot_matrix_trail_3.T @ pos_trial_3[i,:]

    
    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # plot
    # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    fig, axs = plt.subplots(2, 2, figsize=(10, 8)) 
   
    axs[0, 0].scatter(pos_trial_1[:,0], pos_trial_1[:,1], color='green', label='trial 1',marker='o',s=20)
    axs[0, 0].scatter(pos_trial_2[:,0], pos_trial_2[:,1], color='red', label='trial 2',marker='o',s=20)
    axs[0, 0].scatter(pos_trial_3[:,0], pos_trial_3[:,1], color='blue', label='trial 3',marker='o',s=20)
    axs[0, 0].scatter(pos_sim[:,0], pos_sim[:,1], color='black', label='simulation',marker='o',s=20)    

    axs[0, 1].scatter(time_step_1, pos_trial_1[:,1], color='green', label='trial 1',marker='o',s=20)
    axs[0, 1].scatter(time_step_2, pos_trial_2[:,1], color='red', label='trial 2',marker='o',s=20)
    axs[0, 1].scatter(time_step_3, pos_trial_3[:,1], color='blue', label='trial 3',marker='o',s=20)
    axs[0, 1].scatter(time_step_sim, pos_sim[:,1], color='black', label='simulation',marker='o',s=20)

    axs[1, 0].scatter(time_step_1, pos_trial_1[:,0], color='green', label='trial 1',marker='o',s=20)
    axs[1, 0].scatter(time_step_2, pos_trial_2[:,0], color='red', label='trial 2',marker='o',s=20)
    axs[1, 0].scatter(time_step_3, pos_trial_3[:,0], color='blue', label='trial 3',marker='o',s=20)
    axs[1, 0].scatter(time_step_sim, pos_sim[:,0], color='black', label='simulation',marker='o',s=20)

    axs[1, 1].scatter(time_step_1, pos_trial_1[:,2], color='green', label='trial 1',marker='o',s=20)
    axs[1, 1].scatter(time_step_2, pos_trial_2[:,2], color='red', label='trial 2',marker='o',s=20)
    axs[1, 1].scatter(time_step_3, pos_trial_3[:,2], color='blue', label='trial 3',marker='o',s=20)
    axs[1, 1].scatter(time_step_sim, pos_sim[:,2], color='black', label='simulation',marker='o',s=20)

    # axis and label configuration
    axs[0, 0].set_xlim(config['00_range_x'])
    axs[0, 0].set_ylim(config['00_range_y'])
    axs[0, 0].set_xlabel("x_direction(m)")
    axs[0, 0].set_ylabel("y_direction(m)")
    axs[0, 0].set_title(f"{config['label']}")

    axs[0, 1].set_xlabel("time step")
    axs[0, 1].set_ylabel("y_direction(m)")
    axs[0, 1].set_ylim(config['01_range_y'])

    axs[1, 0].set_xlabel("time step")
    axs[1, 0].set_ylabel("x_direction(m)")

    axs[1, 1].set_xlabel("time step")
    axs[1, 1].set_ylabel("z_direction(m)")

    # share configration and legend
    for ax in axs.flatten():
        ax.legend()
        ax.grid(True)
    plt.subplots_adjust(hspace=0.3)


plt.show()

