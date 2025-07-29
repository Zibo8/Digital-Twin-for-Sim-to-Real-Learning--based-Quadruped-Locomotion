import numpy as np
import matplotlib.pyplot as plt
from tra_spline_sim import CubicSpline
from tra_planner_sim import LinearLocalPlanner

pos_sim = np.loadtxt('tra_pos_sim.txt')
obs_sim =  np.loadtxt('tra_obs_sim.txt')
time_step_sim = np.loadtxt('tra_time_step_sim.txt')
# Parameters
bezier_following_steps = 2000
point_array = np.array([[0, 0], [1, 0], [2, -2], [4, 0]], dtype=np.float64)

# Create spline
bez_curve = CubicSpline(p0=point_array[0], p1=point_array[1], p2=point_array[2], p3=point_array[3])
spline_list = [bez_curve]

# Instantiate the planner
llp = LinearLocalPlanner(spline_list,1.0)

# Sample the curve
traj = []
for i in range(bezier_following_steps + 1):
    t = i / bezier_following_steps
    pos = bez_curve.get_position(t)  # Assuming CubicSpline has a method get_point(t)
    traj.append(pos)

traj = np.array(traj)

# Plot the trajectory
plt.figure()
plt.scatter(pos_sim[:,0],pos_sim[:,1],label = 'pos_sim')
plt.plot(traj[:, 0], traj[:, 1], label='Cubic Spline Trajectory')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Trajectory from Cubic Spline')
plt.grid(True)
plt.axis('equal')
plt.legend()
###################

plt.figure()
plt.scatter(time_step_sim,obs_sim[:,10],label = 'command_y')
plt.legend()
plt.show()
