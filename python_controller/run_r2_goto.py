# -*- coding: utf-8 -*-
"""
Created on Wed Mar  2 14:50:46 2022

@author: 75678
"""

import time
import utils
import rospy
import numpy as np
import raven_py_controller

Deg2Rad = np.pi / 180.0
Rad2Deg = 180.0 / np.pi

rospy.init_node('raven_randonm_movement', anonymous=True)

r2py_ctl = raven_py_controller.raven2_py_controller(name_space = ' ', robot_name = 'arm1', grasper_name = 'grasp1')
r2py_ctl.pub_state_command('resume')

target_jpos = np.zeros((16))
target_jpos[1] = 50.0 * Deg2Rad
target_jpos[2] = 70.0 * Deg2Rad
target_jpos[3] = 0.35

r2py_ctl.go_to_jr(target_jpos = target_jpos)

r2py_ctl.pub_state_command('pause')
