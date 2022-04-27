# -*- coding: utf-8 -*-
"""
Created on Wed Apr 20 17:09:52 2022

@author: 75678
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#traj_files_ = ['zigzag_traj_dir_x_0.3333.csv', 'zigzag_traj_dir_y_0.3333.csv', 'zigzag_traj_dir_z_0.3333.csv']
#traj_files_ = ['zigzag_traj_dir_y_0.3333.csv']
#traj_files_ = ['zigzag_traj_dir_x_0.3333.csv', 'zigzag_traj_dir_xy_0.3333.csv', 'zigzag_traj_dir_xyz_0.3333.csv']
#traj_files_ = ['zigzag_traj_dir_x_0.3333.csv', 'zigzag_traj_dir_xy_0.3333.csv', 'zigzag_traj_dir_xyz_0.3333.csv']
traj_files_ = ['zigzag_traj_dir_x_0.3333.csv', 'zigzag_traj_dir_xy_0.3333.csv', 'zigzag_traj_dir_yz_0.3333.csv', 'zigzag_traj_dir_xz_0.3333.csv']
#traj_files_ = ['zigzag_traj_dir_x_0.3333.csv', 'zigzag_traj_dir_yz_0.3333.csv', 'zigzag_traj_dir_xyz_0.3333.csv']

Deg2Rad = np.pi / 180.0
Rad2Deg = 180.0 / np.pi

fig1 = plt.figure(1)
fig1.clf()
ax1 = fig1.add_subplot(111, projection='3d')
ax1.set_xlabel('joint 1(Deg)')
ax1.set_ylabel('joint 2(Deg)')
ax1.set_zlabel('joint 3(m)')

for traj_file in traj_files_:
    traj = np.loadtxt('generated_trajs/' + traj_file, delimiter=',')
    ax1.plot(traj[:,0]*Rad2Deg, traj[:,1]*Rad2Deg, traj[:,2])      
ax1.legend()
plt.title('3D traj in joint space')
plt.show()

