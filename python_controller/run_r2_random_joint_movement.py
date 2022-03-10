"""
Raven 2 Control - Control software for the Raven II robot
Copyright (C) 2005-2022  Andrew Lewis, Yun-Hsuan Su, Haonan Peng, Blake Hannaford,
and the University of Washington BioRobotics Laboratory
This file is part of Raven 2 Control.
Raven 2 Control is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
Raven 2 Control is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.
raven_keyboard_controller.py
Python controller for RAVEN II using CRTK API
date Jan 10, 2022
author Haonan Peng, Yun-Hsuan Su, Andrew Lewis, 
"""

import time
import utils
import rospy
import numpy as np
import raven_py_controller

run_time = 240.0 

switch_smooth_factor = 1000.0 #[IMPT]: This is the smooth factor, it must be positive, and non-zero. Increase this factor will decrease the acceleration of joints after switching direction

Deg2Rad = np.pi / 180.0
Rad2Deg = 180.0 / np.pi

# joint limits in degree and meter(joint 3), this is the soft limit of this controller
limit_joint_1 = np.array([25.0, 55.0]) * Deg2Rad
limit_joint_2 = np.array([75.0, 115.0]) * Deg2Rad
limit_joint_3 = np.array([0.33, 0.39])

# This will be the range of how joint target can be changed 
limit_range_joint_1 = np.array([10, -10]) * Deg2Rad
limit_range_joint_2 = np.array([10, -10]) * Deg2Rad
limit_range_joint_3 = np.array([0.01, -0.01])

vel_limit_joint_1 = np.array([1.5, 2.5]) * Deg2Rad
vel_limit_joint_2 = np.array([1.5, 2.5]) * Deg2Rad
vel_limit_joint_3 = np.array([0.005, 0.010])

rospy.init_node('raven_randonm_movement', anonymous=True)

r2py_ctl = raven_py_controller.raven2_py_controller(name_space = ' ', robot_name = 'arm1', grasper_name = 'grasp1')
r2py_ctl.pub_state_command('resume')

# randomly choose the initial direction of the movement
dir_1 = np.sign(np.random.randn())
dir_2 = np.sign(np.random.randn())
dir_3 = np.sign(np.random.randn())

# Set the initial movement target for each joint
target_joint_1 = limit_joint_1 + np.random.rand() * limit_range_joint_1
target_joint_2 = limit_joint_2 + np.random.rand() * limit_range_joint_2
target_joint_3 = limit_joint_3 + np.random.rand() * limit_range_joint_3

#Set initial joint velocity
vel_joint_1 = np.random.uniform(vel_limit_joint_1[0], vel_limit_joint_1[1])
vel_joint_2 = np.random.uniform(vel_limit_joint_2[0], vel_limit_joint_2[1])
vel_joint_3 = np.random.uniform(vel_limit_joint_3[0], vel_limit_joint_3[1])

# Set joint switch to 0
after_switching_joint_1 = 0.0
after_switching_joint_2 = 0.0
after_switching_joint_3 = 0.0

start_time = time.time()

while time.time() - start_time  <= run_time:
    try:
        cur_jpos_1 = r2py_ctl.measured_jpos[0]
        cur_jpos_2 = r2py_ctl.measured_jpos[1]
        cur_jpos_3 = r2py_ctl.measured_jpos[2]
    except:
        print('waiting for the measure_jp from CRTK')
        continue
    
    # swith the direction after reaching target
    if (dir_1 > 0) & (cur_jpos_1 > target_joint_1[1]):
        dir_1 = dir_1 * -1
        target_joint_1 = limit_joint_1 + np.random.rand() * limit_range_joint_1 # randomly change target
        vel_joint_1 = np.random.uniform(vel_limit_joint_1[0], vel_limit_joint_1[1]) # randomly change velocity   
        after_switching_joint_1 = 0.0   
    if (dir_1 < 0) & (cur_jpos_1 < target_joint_1[0]):
        dir_1 = dir_1 * -1
        target_joint_1 = limit_joint_1 + np.random.rand() * limit_range_joint_1 # randomly change target
        vel_joint_1 = np.random.uniform(vel_limit_joint_1[0], vel_limit_joint_1[1]) # randomly change velocity
        after_switching_joint_1 = 0.0  

    if (dir_2 > 0) & (cur_jpos_2 > target_joint_2[1]):
        dir_2 = dir_2 * -1
        target_joint_2 = limit_joint_2 + np.random.rand() * limit_range_joint_2 # randomly change target
        vel_joint_2 = np.random.uniform(vel_limit_joint_2[0], vel_limit_joint_2[1]) # randomly change velocity
        after_switching_joint_2 = 0.0  
    if (dir_2 < 0) & (cur_jpos_2 < target_joint_2[0]):
        dir_2 = dir_2 * -1
        target_joint_2 = limit_joint_2 + np.random.rand() * limit_range_joint_2 # randomly change target
        vel_joint_2 = np.random.uniform(vel_limit_joint_2[0], vel_limit_joint_2[1]) # randomly change velocity
        after_switching_joint_2 = 0.0  
        
    if (dir_3 > 0) & (cur_jpos_3 > target_joint_3[1]):
        dir_3 = dir_3 * -1
        target_joint_3 = limit_joint_3 + np.random.rand() * limit_range_joint_3 # randomly change target
        vel_joint_3 = np.random.uniform(vel_limit_joint_3[0], vel_limit_joint_3[1]) # randomly change velocity
        after_switching_joint_3 = 0.0  
    if (dir_3 < 0) & (cur_jpos_3 < target_joint_3[0]):
        dir_3 = dir_3 * -1
        target_joint_3 = limit_joint_3 + np.random.rand() * limit_range_joint_3 # randomly change target
        vel_joint_3 = np.random.uniform(vel_limit_joint_3[0], vel_limit_joint_3[1]) # randomly change velocity
        after_switching_joint_3 = 0.0  

            
    cmd = np.zeros((16))
    if dir_1 > 0:
        cmd[1] = dir_1 * np.clip(np.abs(target_joint_1[1] - cur_jpos_1), 0.2*Deg2Rad, vel_joint_1) * 1e-3 * np.clip(after_switching_joint_1/(200+switch_smooth_factor),0,1)
    else:
        cmd[1] = dir_1 * np.clip(np.abs(target_joint_1[0] - cur_jpos_1), 0.2*Deg2Rad, vel_joint_1) * 1e-3 * np.clip(after_switching_joint_1/(200+switch_smooth_factor),0,1)

    if dir_2 > 0:
        cmd[2] = dir_2 * np.clip(np.abs(target_joint_2[1] - cur_jpos_2), 0.2*Deg2Rad, vel_joint_2) * 1e-3 * np.clip(after_switching_joint_2/(200+switch_smooth_factor),0,1)
    else:
        cmd[2] = dir_2 * np.clip(np.abs(target_joint_2[0] - cur_jpos_2), 0.2*Deg2Rad, vel_joint_2) * 1e-3 * np.clip(after_switching_joint_2/(200+switch_smooth_factor),0,1)

    if dir_3 > 0:
        cmd[3] = dir_3 * np.clip(np.abs(target_joint_3[1] - cur_jpos_3), 0.001, vel_joint_3) * 1e-3 * np.clip(after_switching_joint_3/(200+switch_smooth_factor),0,1)
    else:
        cmd[3] = dir_3 * np.clip(np.abs(target_joint_3[0] - cur_jpos_3), 0.001, vel_joint_3) * 1e-3 * np.clip(after_switching_joint_3/(200+switch_smooth_factor),0,1)
    
    # cmd[1] = dir_1 * vel_joint_1 * 1e-3  # old code
    # cmd[2] = dir_2 * vel_joint_2 * 1e-3  # old code
    # cmd[3] = dir_3 * vel_joint_3 * 1e-3  # old code
    r2py_ctl.pub_jr_command(cmd) 

    after_switching_joint_1 += 1
    print(after_switching_joint_1)
    after_switching_joint_2 += 1
    after_switching_joint_3 += 1
        
    
    


r2py_ctl.pub_state_command('pause')
