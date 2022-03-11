# -*- coding: utf-8 -*-
"""
Created on Thu Feb 17 12:12:28 2022

@author: 75678
"""

import numpy as np

Deg2Rad = np.pi / 180.0
Rad2Deg = 180.0 / np.pi


pos_cur = np.array([2,2,2])

traj = np.array([[1,1,1],
                 [2.1,2.1,2.1],
                 [3,3,3]])

idx = np.argmin(np.square(traj[:,0]-pos_cur[0]) + np.square(traj[:,1]-pos_cur[1]) + np.square(traj[:,2]-pos_cur[2]))

pos_tar = traj[idx]
print(idx)
print(pos_tar)
