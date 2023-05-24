# -*- coding: utf-8 -*-
"""
Created on Tue Oct 26 12:00:54 2021

@author: 75678
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from utils_transform import Trans

traj_type = "dir_yz"
scar_x = 0
scar_y = 0.1666
scar_z = 0.1666

scar_l = 0.001



limit_joint_1 = [20, 60]  # joint 1, in deg
limit_joint_2 = [55, 110]  # joint 2, in deg
limit_joint_3 = [0.33, 0.42]  # joint 3, in m

#limit_joint_1 = [0.0, 1.0]  # for debug
#limit_joint_2 = [0.0, 1.0]  # for debug
#limit_joint_3 = [0.0, 1.0]  # for debug

# factor_cube_diag = [0.0, 1.0] 
# factor_edge = [0.5 - 0.5*1.0/np.sqrt(3), 0.5 + 0.5*1.0/np.sqrt(3)]
# factor_square_diag = [0.5 - 0.5*np.sqrt(2)*(factor_edge[1]-factor_edge[0]), 0.5 + 0.5*np.sqrt(2)*(factor_edge[1]-factor_edge[0])]

factor_edge = 1 / np.sqrt(3)
factor_square_diag = factor_edge * np.sqrt(2)
factor_cube_diag = 1

print(factor_cube_diag)
print(factor_edge)
print(factor_square_diag)

scale_x = [0,1] 
scale_y = [0,1] 
scale_z = [0,1] 

file_name = 'generated_trajs/zigzag_traj_' + traj_type + '_' + str(scar_y) + '.csv'

sqrt2 = np.sqrt(2)
sqrt3 = np.sqrt(3)

tar_x = 1
tar_y = scar_y
tar_z = scar_z

dir_x = 1
dir_y = 1
dir_z = 1

dis_x = 0
dis_y = 0
dis_z = 0

pos_x = 0
pos_y = 0
pos_z = 0

move_x = True
move_y = False
move_z = False

finish = False

Deg2Rad = np.pi / 180.0
Rad2Deg = 180.0 / np.pi

traj = np.array([[0,0,0]])
point = np.array([0,0,0])

while finish == False:
    # print(point)
    # print(pos_y)
    
    if move_x == True:
        point = point + np.array([scar_l,0,0]) * dir_x
        traj = np.append(traj, [point], axis=0)
        dis_x = dis_x + scar_l
        if (pos_y >= 1) & (dis_x >= 1):
            move_x = False
            move_y = False
            move_z = True
            dis_x = 0
            dir_x = dir_x * -1
            
            dis_y = 0
            pos_y = 0
            dir_y = dir_y * -1
            
        elif dis_x >= 1:
            move_x = False
            move_y = True
            dis_x = 0
            dir_x = dir_x * -1
                 
    elif move_y == True:
        point = point + np.array([0,scar_l,0]) * dir_y
        traj = np.append(traj, [point], axis=0)
        dis_y = dis_y + scar_l
        pos_y = pos_y + scar_l
        if dis_y >= scar_y:
            move_x = True
            move_y = False
            dis_y = 0
        
    elif move_z == True:
        point = point + np.array([0,0,scar_l]) * dir_z
        traj = np.append(traj, [point], axis=0)
        dis_z = dis_z + scar_l
        pos_z = pos_z + scar_l
        if dis_z >= scar_z:
            move_x = True
            move_y = False
            move_z = False
            dis_z = 0
    #print(point)
    if pos_z > 1.05:
            finish == True
            break
        
# M_trans = np.dot(Trans('x', 0, [-0.5,-0.5,0]), np.dot(Trans('z', 45, [0,0,0]), Trans('x', 0, [0.5,0.5,0])))
# M_trans = Trans('z', 45, [0,0,0])
# M_trans = np.dot(Trans('z', 45, [0,0,0]), Trans('x', 0, [-0.5,-0.5,-0.5]))
# M_trans = np.dot(np.dot(Trans('z', 45, [0,0,0]), Trans('x', 0, [-0.5,-0.5,-0.5])), Trans('y', 45, [0,0,0]))

traj = np.hstack((traj, np.ones((traj.shape[0], 1))))

if traj_type == 'dir_x':
    scale_x = [0.5*(limit_joint_1[1]+limit_joint_1[0]) - 0.5*factor_edge*(limit_joint_1[1]-limit_joint_1[0]), 
                0.5*(limit_joint_1[1]+limit_joint_1[0]) + 0.5*factor_edge*(limit_joint_1[1]-limit_joint_1[0])] 
    scale_y = [0.5*(limit_joint_2[1]+limit_joint_2[0]) - 0.5*factor_edge*(limit_joint_2[1]-limit_joint_2[0]), 
                0.5*(limit_joint_2[1]+limit_joint_2[0]) + 0.5*factor_edge*(limit_joint_2[1]-limit_joint_2[0])] 
    scale_z = [0.5*(limit_joint_3[1]+limit_joint_3[0]) - 0.5*factor_edge*(limit_joint_3[1]-limit_joint_3[0]), 
                0.5*(limit_joint_3[1]+limit_joint_3[0]) + 0.5*factor_edge*(limit_joint_3[1]-limit_joint_3[0])] 
    traj = traj

if traj_type == 'dir_y':
    scale_x = [0.5*(limit_joint_1[1]+limit_joint_1[0]) - 0.5*factor_edge*(limit_joint_1[1]-limit_joint_1[0]), 
                0.5*(limit_joint_1[1]+limit_joint_1[0]) + 0.5*factor_edge*(limit_joint_1[1]-limit_joint_1[0])] 
    scale_y = [0.5*(limit_joint_2[1]+limit_joint_2[0]) - 0.5*factor_edge*(limit_joint_2[1]-limit_joint_2[0]), 
                0.5*(limit_joint_2[1]+limit_joint_2[0]) + 0.5*factor_edge*(limit_joint_2[1]-limit_joint_2[0])] 
    scale_z = [0.5*(limit_joint_3[1]+limit_joint_3[0]) - 0.5*factor_edge*(limit_joint_3[1]-limit_joint_3[0]), 
                0.5*(limit_joint_3[1]+limit_joint_3[0]) + 0.5*factor_edge*(limit_joint_3[1]-limit_joint_3[0])] 
    
    M_trans = np.dot(Trans('x', 0, [0.5 , 0.5,0.5]), np.dot(Trans('z', 90, [0,0,0]), Trans('z', 0, [-0.5,-0.5,-0.5])))
    traj_rot = np.dot(M_trans,traj.T).T
    traj_rot[:,0] = traj_rot[:,0] 
    traj_rot[:,1] = traj_rot[:,1] 
    traj_rot[:,2] = traj_rot[:,2] 
    traj = traj_rot

if traj_type == 'dir_z':
    scale_x = [0.5*(limit_joint_1[1]+limit_joint_1[0]) - 0.5*factor_edge*(limit_joint_1[1]-limit_joint_1[0]), 
                0.5*(limit_joint_1[1]+limit_joint_1[0]) + 0.5*factor_edge*(limit_joint_1[1]-limit_joint_1[0])] 
    scale_y = [0.5*(limit_joint_2[1]+limit_joint_2[0]) - 0.5*factor_edge*(limit_joint_2[1]-limit_joint_2[0]), 
                0.5*(limit_joint_2[1]+limit_joint_2[0]) + 0.5*factor_edge*(limit_joint_2[1]-limit_joint_2[0])] 
    scale_z = [0.5*(limit_joint_3[1]+limit_joint_3[0]) - 0.5*factor_edge*(limit_joint_3[1]-limit_joint_3[0]), 
                0.5*(limit_joint_3[1]+limit_joint_3[0]) + 0.5*factor_edge*(limit_joint_3[1]-limit_joint_3[0])] 
    
    M_trans = np.dot(Trans('x', 0, [0.5 , 0.5,0.5]), np.dot(Trans('y', 90, [0,0,0]), Trans('z', 0, [-0.5,-0.5,-0.5])))
    traj_rot = np.dot(M_trans,traj.T).T
    traj_rot[:,0] = traj_rot[:,0] 
    traj_rot[:,1] = traj_rot[:,1] 
    traj_rot[:,2] = traj_rot[:,2] 
    traj = traj_rot
    
if traj_type == 'dir_xy':
    scale_x = [0.5*(limit_joint_1[1]+limit_joint_1[0]) - 0.5*factor_square_diag*(limit_joint_1[1]-limit_joint_1[0]), 
                0.5*(limit_joint_1[1]+limit_joint_1[0]) + 0.5*factor_square_diag*(limit_joint_1[1]-limit_joint_1[0])] 
    scale_y = [0.5*(limit_joint_2[1]+limit_joint_2[0]) - 0.5*factor_square_diag*(limit_joint_2[1]-limit_joint_2[0]), 
                0.5*(limit_joint_2[1]+limit_joint_2[0]) + 0.5*factor_square_diag*(limit_joint_2[1]-limit_joint_2[0])] 
    scale_z = [0.5*(limit_joint_3[1]+limit_joint_3[0]) - 0.5*factor_edge*(limit_joint_3[1]-limit_joint_3[0]), 
                0.5*(limit_joint_3[1]+limit_joint_3[0]) + 0.5*factor_edge*(limit_joint_3[1]-limit_joint_3[0])] 
    
    M_trans = np.dot(Trans('x', 0, [0.5 * sqrt2, 0.5* sqrt2,0.5]), np.dot(Trans('z', 45, [0,0,0]), Trans('z', 0, [-0.5,-0.5,-0.5])))
    traj_rot = np.dot(M_trans,traj.T).T
    traj_rot[:,0] = traj_rot[:,0] / sqrt2
    traj_rot[:,1] = traj_rot[:,1] / sqrt2
    traj_rot[:,2] = traj_rot[:,2] 
    traj = traj_rot

if traj_type == 'dir_yz':
    scale_x = [0.5*(limit_joint_1[1]+limit_joint_1[0]) - 0.5*factor_edge*(limit_joint_1[1]-limit_joint_1[0]), 
                0.5*(limit_joint_1[1]+limit_joint_1[0]) + 0.5*factor_edge*(limit_joint_1[1]-limit_joint_1[0])] 
    scale_y = [0.5*(limit_joint_2[1]+limit_joint_2[0]) - 0.5*factor_square_diag*(limit_joint_2[1]-limit_joint_2[0]), 
                0.5*(limit_joint_2[1]+limit_joint_2[0]) + 0.5*factor_square_diag*(limit_joint_2[1]-limit_joint_2[0])] 
    scale_z = [0.5*(limit_joint_3[1]+limit_joint_3[0]) - 0.5*factor_square_diag*(limit_joint_3[1]-limit_joint_3[0]), 
                0.5*(limit_joint_3[1]+limit_joint_3[0]) + 0.5*factor_square_diag*(limit_joint_3[1]-limit_joint_3[0])] 
    
    M_trans = np.dot(Trans('x', 0, [0.5 , 0.5* sqrt2,0.5* sqrt2]), np.dot(Trans('x', 45, [0,0,0]), np.dot(Trans('z', 90, [0,0,0]), Trans('z', 0, [-0.5,-0.5,-0.5]))))
    traj_rot = np.dot(M_trans,traj.T).T
    traj_rot[:,0] = traj_rot[:,0] 
    traj_rot[:,1] = traj_rot[:,1] / sqrt2
    traj_rot[:,2] = traj_rot[:,2] / sqrt2
    traj = traj_rot

if traj_type == 'dir_xz':
    scale_x = [0.5*(limit_joint_1[1]+limit_joint_1[0]) - 0.5*factor_square_diag*(limit_joint_1[1]-limit_joint_1[0]), 
                0.5*(limit_joint_1[1]+limit_joint_1[0]) + 0.5*factor_square_diag*(limit_joint_1[1]-limit_joint_1[0])] 
    scale_y = [0.5*(limit_joint_2[1]+limit_joint_2[0]) - 0.5*factor_edge*(limit_joint_2[1]-limit_joint_2[0]), 
                0.5*(limit_joint_2[1]+limit_joint_2[0]) + 0.5*factor_edge*(limit_joint_2[1]-limit_joint_2[0])] 
    scale_z = [0.5*(limit_joint_3[1]+limit_joint_3[0]) - 0.5*factor_square_diag*(limit_joint_3[1]-limit_joint_3[0]), 
                0.5*(limit_joint_3[1]+limit_joint_3[0]) + 0.5*factor_square_diag*(limit_joint_3[1]-limit_joint_3[0])] 
    
    M_trans = np.dot(Trans('x', 0, [0.5 * sqrt2, 0.5, 0.5* sqrt2]), np.dot(Trans('y', 45, [0,0,0]), Trans('z', 0, [-0.5,-0.5,-0.5])))
    traj_rot = np.dot(M_trans,traj.T).T
    traj_rot[:,0] = traj_rot[:,0] / sqrt2
    traj_rot[:,1] = traj_rot[:,1] 
    traj_rot[:,2] = traj_rot[:,2] / sqrt2
    traj = traj_rot
    
#if traj_type == 'dir_xyz':
#    scale_x = [0.5*(limit_joint_1[1]+limit_joint_1[0]) - 0.5*factor_cube_diag*(limit_joint_1[1]-limit_joint_1[0]), 
#                0.5*(limit_joint_1[1]+limit_joint_1[0]) + 0.5*factor_cube_diag*(limit_joint_1[1]-limit_joint_1[0])] 
#    scale_y = [0.5*(limit_joint_2[1]+limit_joint_2[0]) - 0.5*factor_cube_diag*(limit_joint_2[1]-limit_joint_2[0]), 
#                0.5*(limit_joint_2[1]+limit_joint_2[0]) + 0.5*factor_cube_diag*(limit_joint_2[1]-limit_joint_2[0])] 
#    scale_z = [0.5*(limit_joint_3[1]+limit_joint_3[0]) - 0.5*factor_cube_diag*(limit_joint_3[1]-limit_joint_3[0]), 
#                0.5*(limit_joint_3[1]+limit_joint_3[0]) + 0.5*factor_cube_diag*(limit_joint_3[1]-limit_joint_3[0])] 
#    
#    M_trans = np.dot(Trans('x', 0, [0.5*sqrt3,0.5*sqrt3,0.5*sqrt3]), np.dot(Trans('x', 45, [0,0,0]), np.dot(Trans('z', 45, [0,0,0]), Trans('z', 0, [-0.5,-0.5,-0.5]))))
#    traj_rot = np.dot(M_trans,traj.T).T
#    traj_rot[:,0] = traj_rot[:,0] / sqrt3
#    traj_rot[:,1] = traj_rot[:,1] / sqrt3
#    traj_rot[:,2] = traj_rot[:,2] / sqrt3
#    
#    traj_rot[:,0] = (traj_rot[:,0] - 0.09093521295520893)/(0.9082482904638634-0.09093521295520893) # this is a special compensation, only for diamond traj pose
#    traj = traj_rot

if traj_type == 'dir_xyz':
    scale_x = [0.5*(limit_joint_1[1]+limit_joint_1[0]) - 0.5*factor_cube_diag*(limit_joint_1[1]-limit_joint_1[0]), 
                0.5*(limit_joint_1[1]+limit_joint_1[0]) + 0.5*factor_cube_diag*(limit_joint_1[1]-limit_joint_1[0])] 
    scale_y = [0.5*(limit_joint_2[1]+limit_joint_2[0]) - 0.5*factor_cube_diag*(limit_joint_2[1]-limit_joint_2[0]), 
                0.5*(limit_joint_2[1]+limit_joint_2[0]) + 0.5*factor_cube_diag*(limit_joint_2[1]-limit_joint_2[0])] 
    scale_z = [0.5*(limit_joint_3[1]+limit_joint_3[0]) - 0.5*factor_cube_diag*(limit_joint_3[1]-limit_joint_3[0]), 
                0.5*(limit_joint_3[1]+limit_joint_3[0]) + 0.5*factor_cube_diag*(limit_joint_3[1]-limit_joint_3[0])] 
    
    M_trans = np.dot(Trans('x', 0, [0.5*sqrt3,0.5*sqrt3,0.5*sqrt3]), np.dot(Trans('y', 45, [0,0,0]), np.dot(Trans('x', 45, [0,0,0]), np.dot(Trans('z', 45, [0,0,0]), Trans('z', 0, [-0.5,-0.5,-0.5])))))
    traj_rot = np.dot(M_trans,traj.T).T
    traj_rot[:,0] = traj_rot[:,0] / sqrt3
    traj_rot[:,1] = traj_rot[:,1] / sqrt3
    traj_rot[:,2] = traj_rot[:,2] / sqrt3
    traj_rot[:,0] = (traj_rot[:,0] - 0.0668181961291)/(0.933590052161-0.0668181961291) # this is a special compensation, only for diamond traj pose
    traj_rot[:,2] = (traj_rot[:,2] - 0.0669872981078)/(0.934575650721-0.0669872981078) # this is a special compensation, only for diamond traj pose
    traj = traj_rot

traj[:,0] = traj[:,0]*(scale_x[1]-scale_x[0]) + scale_x[0]
traj[:,1] = traj[:,1]*(scale_y[1]-scale_y[0]) + scale_y[0]
traj[:,2] = traj[:,2]*(scale_z[1]-scale_z[0]) + scale_z[0]

print('Trajectory Summary:')
print('Joint 1| Max: ' + str(np.max(traj[:,0])) + ' Min: ' +  str(np.min(traj[:,0])))
print('Joint 2| Max: ' + str(np.max(traj[:,1])) + ' Min: ' +  str(np.min(traj[:,1])))
print('Joint 3 Max: ' + str(np.max(traj[:,2])) + ' Min: ' +  str(np.min(traj[:,2])))

# traj_rot = np.dot(M_trans,traj.T).T
# print(M_trans)
# traj = traj_rot

traj[:,0] = traj[:,0] * Deg2Rad
traj[:,1] = traj[:,1] * Deg2Rad

np.savetxt(file_name, traj[:,0:3], delimiter=',')
        
print(np.shape(traj))

#fig = plt.figure()
#ax = plt.axes(projection='3d')
#ax.plot3D(traj[:,0] * Rad2Deg, traj[:,1] * Rad2Deg, traj[:,2])
#ax.set_xlabel('Joint 1 (deg)')
#ax.set_ylabel('Joint 2 (deg)')
#ax.set_zlabel('Joint 3 (m)')
#plt.title('Sparcity- Joint 1: ' + str(scar_x) + ' Joint 2: ' + str(scar_y) + ' Joint 3: ' + str(scar_z))
#plt.savefig(file_name[:-3] + 'png')

fig1 = plt.figure(1)
fig1.clf()
ax1 = fig1.add_subplot(111, projection='3d')
#ax1 = fig1.gca(projection='3d')
ax1.set_xlabel('joint 1(Deg)')
ax1.set_ylabel('joint 2(Deg)')
ax1.set_zlabel('joint 3(m)')
ax1.plot(traj[:,0]*Rad2Deg, traj[:,1]*Rad2Deg, traj[:,2])      
ax1.legend()
plt.title('3D traj in joint space')
plt.savefig(file_name[:-3] + 'png')


