# -*- coding: utf-8 -*-
"""
Created on Tue Oct 26 12:00:54 2021

@author: 75678
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from utils_transform import Trans

traj_type = "dir_xz"
scar_x = 0
scar_y = 0.3333
scar_z = 0.3333

scar_l = 0.001



cube_limit_joint_1 = [20, 60]  # joint 1, in deg
cube_limit_joint_2 = [55, 110]  # joint 2, in deg
cube_limit_joint_3 = [0.33, 0.42]  # joint 3, in m

range_joint_1 = cube_limit_joint_1[1]-cube_limit_joint_1[0]
range_joint_2 = cube_limit_joint_2[1]-cube_limit_joint_2[0]
range_joint_3 = cube_limit_joint_3[1]-cube_limit_joint_3[0]

factor_edge = 1 / np.sqrt(3)
factor_square = factor_edge * np.sqrt(2)
factor_cube = 1

edge_limit_joint_1 = [cube_limit_joint_1[0]+0.5*(1-factor_edge)*range_joint_1, cube_limit_joint_1[1]-0.5*(1-factor_edge)*range_joint_1]
edge_limit_joint_2 = [cube_limit_joint_2[0]+0.5*(1-factor_edge)*range_joint_2, cube_limit_joint_2[1]-0.5*(1-factor_edge)*range_joint_2]
edge_limit_joint_3 = [cube_limit_joint_3[0]+0.5*(1-factor_edge)*range_joint_3, cube_limit_joint_3[1]-0.5*(1-factor_edge)*range_joint_3]

square_limit_joint_1 = [cube_limit_joint_1[0]+0.5*(1-factor_square)*range_joint_1, cube_limit_joint_1[1]-0.5*(1-factor_square)*range_joint_1]
square_limit_joint_2 = [cube_limit_joint_2[0]+0.5*(1-factor_square)*range_joint_2, cube_limit_joint_2[1]-0.5*(1-factor_square)*range_joint_2]
square_limit_joint_3 = [cube_limit_joint_3[0]+0.5*(1-factor_square)*range_joint_3, cube_limit_joint_3[1]-0.5*(1-factor_square)*range_joint_3]

print('-----------------------------------')
print('Joint 1 Edge:')
print(edge_limit_joint_1)
print('Joint 1 Square:')
print(square_limit_joint_1)
print('Joint 1 Cube:')
print(cube_limit_joint_1)
print('-----------------------------------')
print('Joint 2 Edge:')
print(edge_limit_joint_2)
print('Joint 2 Square:')
print(square_limit_joint_2)
print('Joint 2 Cube:')
print(cube_limit_joint_2)
print('-----------------------------------')
print('Joint 3 Edge:')
print(edge_limit_joint_3)
print('Joint 3 Square:')
print(square_limit_joint_3)
print('Joint 3 Cube:')
print(cube_limit_joint_3)
