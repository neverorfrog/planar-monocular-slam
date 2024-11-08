import numpy as np
import matplotlib.pyplot as plt

trajectory = np.loadtxt('data/trajectory.dat')

odom_traj = np.stack((trajectory[:,1], trajectory[:,2], trajectory[:,3]), axis = 1)
gt_traj = np.stack((trajectory[:,4], trajectory[:,5], trajectory[:,6]), axis = 1)
plt.plot(odom_traj[:,0], odom_traj[:,1], 'bo', label = 'odometry')
plt.plot(gt_traj[:,0], gt_traj[:,1], 'ro',label = 'ground truth')
plt.legend()
plt.show()