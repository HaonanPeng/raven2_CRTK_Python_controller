# -*- coding: utf-8 -*-
"""
Created on Thu Mar 10 15:44:22 2022
@author: 75678
"""
import sys
import time
import rospy
import numpy as np
import raven_py_controller
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

smooth_factor = 100 # [IMPT]: Must be positive. This is the factor to prevent the control command to be unstable. A larger factor could result in more smooth control commands, but the accuracy of following can be also decreased.
max_step_dis = 50 # [IMPT]: Must be positive, this is the max distance when finding the nearest point in the target trajectory
min_step_dis = 20 # [IMPT]: Must be positive, this is the min distance when finding the nearest point in the target trajectory
spd_factor = 2.0 # [IMPT]: Must be positive

show_plot = False # If ture, a sub-realtime plot will be shown when running

trajctory_file = 'trajectory/zigzag_traj_dir_x_0.3333.csv'

Deg2Rad = np.pi / 180.0
Rad2Deg = 180.0 / np.pi

intv_record = 0.1
intv_plot = 2

time_last_record = 0
time_last_plot = 0

limit_joint_1 = np.array([15.0, 70.0]) * Deg2Rad
limit_joint_2 = np.array([50.0, 115.0]) * Deg2Rad # should  be 115 deg
limit_joint_3 = np.array([0.32, 0.43])

velocity_joint_1 = 3.0 * Deg2Rad # max degree/s
velocity_joint_2 = 3.0 * Deg2Rad # max degree/s
velocity_joint_3 = 0.01 # max m/s

traj = np.loadtxt(trajctory_file,  delimiter=',')
shape_traj = traj.shape

# check if the traj exceed joint limits
if np.min(traj[:,0]) < limit_joint_1[0]:
    print('Joint 1 trajectory exceed min: ' + str(np.min(traj[:,0])*Rad2Deg) + ' Deg')
    sys.exit('Exiting')
if np.max(traj[:,0]) > limit_joint_1[1]:
    print('Joint 1 trajectory exceed max: ' + str(np.max(traj[:,0])*Rad2Deg) + ' Deg')
    sys.exit('Exiting')
if np.min(traj[:,1]) < limit_joint_2[0]:
    print('Joint 2 trajectory exceed min: ' + str(np.min(traj[:,1])*Rad2Deg) + ' Deg')
    sys.exit('Exiting')
if np.max(traj[:,1]) > limit_joint_2[1]:
    print('Joint 2 trajectory exceed max: ' + str(np.max(traj[:,1])*Rad2Deg) + ' Deg')
    sys.exit('Exiting')
if np.min(traj[:,2]) < limit_joint_3[0]:
    print('Joint 3 trajectory exceed min: ' + str(np.min(traj[:,2])) + ' m')
    sys.exit('Exiting')
if np.max(traj[:,2]) > limit_joint_3[1]:
    print('Joint 3 trajectory exceed max: ' + str(np.max(traj[:,2])) + ' m')
    sys.exit('Exiting')

tail = np.ones((smooth_factor + 1, 3))
traj = np.vstack((traj, tail))

rospy.init_node('raven_trajectory_follower', anonymous=True)
r2py_ctl = raven_py_controller.raven2_py_controller(name_space = ' ', robot_name = 'arm1', grasper_name = 'grasp1')
r2py_ctl.pub_state_command('resume')

rec_jpos = np.zeros((1,3))

# first, go to start point
print('Going to the start point of the trajectory')
target_jpos = np.zeros((16))
target_jpos[1] = traj[0,0]
target_jpos[2] = traj[0,1]
target_jpos[3] = traj[0,2]
r2py_ctl.go_to_jr(target_jpos = target_jpos)
print('Reach the start point, now follow the trajectory')

start_time = time.time()
idx_pre = 0
moving = True
while moving == True:
    pos_cur = np.array([r2py_ctl.measured_jpos[0],r2py_ctl.measured_jpos[1],r2py_ctl.measured_jpos[2]])
    idx = np.argmin(np.square(traj[:,0]-pos_cur[0]) + np.square(traj[:,1]-pos_cur[1]) + np.square(traj[:,2]-pos_cur[2]))
    
    if idx >= (shape_traj[0] - 200):
        end_time = time.time()-start_time

        for i in range(0,2000):
            cmd = np.zeros((16))
            r2py_ctl.pub_jr_command(cmd * 1e-3) 

        target_jpos = np.zeros((16))
        target_jpos[1] = 30 * Deg2Rad
        target_jpos[2] = 75 * Deg2Rad
        target_jpos[3] = 0.300
        r2py_ctl.go_to_jr(target_jpos = target_jpos)        
        r2py_ctl.pub_state_command('pause')
        print('Trajectory following finished, time used(sec):')
        print(end_time)
        sys.exit('Trajectory finished, existing')
    if (idx - idx_pre) >= max_step_dis:
        idx = idx + max_step_dis
    elif (idx - idx_pre) <= min_step_dis:
        idx = idx + min_step_dis
    
    pos_tar = traj[idx+smooth_factor]
    #print(idx)
    
    cmd = np.zeros((16))
    cmd[1] = np.clip(spd_factor*(pos_tar[0] - pos_cur[0]), -velocity_joint_1, velocity_joint_1)
    cmd[2] = np.clip(spd_factor*(pos_tar[1] - pos_cur[1]), -velocity_joint_2, velocity_joint_2)
    cmd[3] = np.clip(spd_factor*(pos_tar[2] - pos_cur[2]), -velocity_joint_3, velocity_joint_3)
    print('------------------------------')
    print(idx)
    print(cmd[1] * Rad2Deg)
    print(cmd[2] * Rad2Deg)
    print(cmd[3])
    if np.abs(cmd[1] * Rad2Deg) < 0.2:
        cmd[1] = 0

    if np.abs(cmd[2] * Rad2Deg) < 0.2:
        cmd[2] = 0

    if np.abs(cmd[3]) < 0.001:
        cmd[3] = 0

    #print(cmd)
    r2py_ctl.pub_jr_command(cmd * 1e-3)     
    
    idx_pre = idx

    if show_plot == True:
        if time.time()-time_last_record > intv_record:
            rec_jpos = np.append(rec_jpos, np.array([pos_cur]), axis = 0)
            time_last_record = time.time()
        if time.time()-time_last_plot > intv_plot:
            fig1 = plt.figure(1)
            fig1.clf()
            ax1 = fig1.add_subplot(111, projection='3d')
            #ax1 = fig1.gca(projection='3d')
            ax1.set_xlabel('joint 1(Deg)')
            ax1.set_ylabel('joint 2(Deg)')
            ax1.set_zlabel('joint 3(m)')
            ax1.plot(rec_jpos[1:,0]*Rad2Deg, rec_jpos[1:,1]*Rad2Deg, rec_jpos[1:,2], c='b')
            ax1.plot(traj[:,0]*Rad2Deg, traj[:,1]*Rad2Deg, traj[1:,2], c='r')
            ax1.scatter(rec_jpos[-1,1]*Rad2Deg, rec_jpos[-1,2]*Rad2Deg, rec_jpos[-1,3], label=str("%.4f" % (rec_jpos[-1,1]*Rad2Deg)) + ', ' + str("%.4f" % (rec_jpos[-1,2]*Rad2Deg)) + ', ' + str("%.4f" % (rec_jpos[-1,3])), c='y')         
            ax1.legend()
            plt.title('3D traj in joint space')
            plt.show(block = False)
            time_last_plot = time.time()
